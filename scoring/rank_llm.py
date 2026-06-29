#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Ranking Pipeline for ChronoBench
=================================

1. Reads and merges two evaluation CSVs:
   - combined_evaluation_scores.csv (LLM-as-judge scores)
   - evaluation_results.csv (CodeBLEU/ROUGE metrics)
   
2. Creates a unified dataset with all metrics

3. Ranks LLMs by multiple methods:
   - Per-file ranking
   - Consensus ranking (z-score based)

4. Outputs:
   - all_metrics_merged.csv: Complete merged dataset
   - llm_all_metrics_with_rank.csv: Per-model aggregated metrics with rank
   - consensus_llm_rankings.csv: Final consensus ranking
   - Per-file rankings
   - Top-10 plot

Author: ChronoBench Team
"""

from pathlib import Path
import re
import numpy as np
import pandas as pd

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

# ---------- Input/Output Paths ----------
COMBINED_SCORES = PROJECT_ROOT / "output_llms" / "combined_evaluation_scores.csv"
EVAL_RESULTS = PROJECT_ROOT / "metrics" / "evaluation_results.csv"
OUT_DIR = SCRIPT_DIR / "out"
MAKE_PLOT = True

# Optional plotting
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except Exception:
    MATPLOTLIB_OK = False


# =============================================================================
# STEP 1: Merge the two CSV files
# =============================================================================
def merge_evaluation_files() -> pd.DataFrame:
    """
    Merge combined_evaluation_scores.csv and evaluation_results.csv
    into a single DataFrame with all metrics.
    """
    print("=" * 60)
    print("Step 1: Merging evaluation files...")
    print("=" * 60)
    
    # Load both files
    df_scores = pd.read_csv(COMBINED_SCORES)
    df_metrics = pd.read_csv(EVAL_RESULTS)
    
    print(f"\n  combined_evaluation_scores.csv: {len(df_scores)} rows, {len(df_scores.columns)} cols")
    print(f"  evaluation_results.csv: {len(df_metrics)} rows, {len(df_metrics.columns)} cols")
    
    # Normalize df_metrics keys (round_1 -> 1)
    df_metrics['round_norm'] = df_metrics['round'].str.replace('round_', '')
    df_metrics['_key'] = df_metrics['model'] + '|' + df_metrics['system'] + '|' + df_metrics['round_norm']
    
    # Normalize df_scores keys (first -> 1)
    round_map = {'first': '1', 'second': '2', 'third': '3'}
    df_scores['round_norm'] = df_scores['Round'].map(round_map)
    df_scores['_key'] = df_scores['Test Model'] + '|' + df_scores['System'] + '|' + df_scores['round_norm']
    
    # Rename df_scores columns
    # Internal snake_case keys; headers come from chronobench.score.
    df_scores_renamed = df_scores.rename(columns={
        'Test Model': 'model',
        'System': 'system',
        'Round': 'round_name',
        'Score API': 'score_document',
        'Score Reference': 'score_reference',
        'Score Reference API': 'score_reference_document'
    })
    
    # Select columns to merge from df_scores
    score_cols = ['_key', 'score_document', 'score_reference', 'score_reference_document']
    df_scores_to_merge = df_scores_renamed[score_cols]
    
    # Merge on key
    merged = df_metrics.merge(df_scores_to_merge, on='_key', how='outer')
    
    # Clean up helper columns
    merged = merged.drop(columns=['_key', 'round_norm'], errors='ignore')
    
    # Reorder columns
    id_cols = ['model', 'system', 'round']
    metric_cols = [c for c in merged.columns if c not in id_cols]
    merged = merged[id_cols + metric_cols]
    
    # Sort
    round_order = {'round_1': 1, 'round_2': 2, 'round_3': 3}
    merged['_sort'] = merged['round'].map(round_order)
    merged = merged.sort_values(['model', 'system', '_sort']).drop(columns=['_sort'])
    merged = merged.reset_index(drop=True)
    
    print(f"\n  Merged: {len(merged)} rows, {len(merged.columns)} columns")
    
    return merged


# =============================================================================
# STEP 2: Helper functions for ranking
# =============================================================================
def normalize_cols(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [c.strip() for c in df.columns]
    return df


def detect_model_col(df: pd.DataFrame) -> str:
    lower = {c.lower(): c for c in df.columns}
    for key in ["model", "test model", "llm", "model_name"]:
        if key in lower:
            return lower[key]
    return df.columns[0]


def get_numeric_and_score_like(df: pd.DataFrame):
    numeric_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
    
    # Exclude flag-like columns
    flag_patterns = [r"compile", r"pass", r"success", r"fail", r"error", r"warning"]
    flag_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in flag_patterns)]
    
    # Score-like columns
    score_patterns = [
        r"score", r"bleu", r"rouge", r"codebleu",
        r"exact", r"\bf1\b", r"\bem\b", r"accuracy", r"\bacc\b",
        r"mcc", r"pearson", r"spearman", r"reference", r"document"
    ]
    score_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in score_patterns)]
    score_like = [c for c in score_like if c not in flag_like]
    
    return numeric_cols, score_like


def pick_primary_metric(df: pd.DataFrame):
    prefs = [
        r"score.*reference.*document", r"score.*reference\s*document",
        r"score reference document", r"score_reference_document",
        r"score.*reference", r"score.*document"
    ]
    for pat in prefs:
        for c in df.columns:
            if re.search(pat, c, flags=re.I) and pd.api.types.is_numeric_dtype(df[c]):
                return c, False
    return "__mean_scores__", True


def zscore(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    mu, sd = s.mean(), s.std(ddof=0)
    if sd == 0 or np.isnan(sd):
        return pd.Series(np.zeros(len(s)), index=s.index)
    return (s - mu) / sd


def minmax01(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    mn, mx = s.min(), s.max()
    if mx == mn:
        return pd.Series(np.ones(len(s)) * 0.5, index=s.index)
    return (s - mn) / (mx - mn)


# =============================================================================
# STEP 3: Rank one file
# =============================================================================
def rank_one_file(df: pd.DataFrame, model_col: str, primary_metric: str, 
                  use_mean_of_scores: bool, score_like: list) -> pd.DataFrame:
    agg = df.copy()
    if use_mean_of_scores:
        agg[primary_metric] = agg[score_like].mean(axis=1, skipna=True)
    
    per_model = (
        agg.groupby(model_col)[primary_metric]
        .mean()
        .sort_values(ascending=False)
        .reset_index()
        .rename(columns={primary_metric: "RankMetric"})
    )
    per_model["Rank"] = np.arange(1, len(per_model) + 1)
    
    # Attach common metrics
    extras = []
    for pat in [r"score_document", r"score_reference", r"score_reference_document",
                r"codebleu", r"rouge1", r"rouge2", r"rougeL", r"rougeLsum"]:
        for c in df.columns:
            if re.search(pat, c, flags=re.I) and pd.api.types.is_numeric_dtype(df[c]) and c not in extras:
                extras.append(c)
    
    if extras:
        extra_means = df.groupby(model_col).agg({m: "mean" for m in extras}).reset_index()
        per_model = per_model.merge(extra_means, on=model_col, how="left")
    
    cols = ["Rank", model_col, "RankMetric"] + [c for c in per_model.columns if c not in {"Rank", model_col, "RankMetric"}]
    return per_model[cols]


# =============================================================================
# STEP 4: Build consensus ranking from merged data
# =============================================================================
def build_rankings(merged_df: pd.DataFrame) -> dict:
    """
    Build rankings from the merged DataFrame.
    Returns dict with ranking DataFrames and info.
    """
    print("\n" + "=" * 60)
    print("Step 2: Building rankings...")
    print("=" * 60)
    
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    
    # Save merged data
    merged_path = OUT_DIR / "all_metrics_merged.csv"
    merged_df.to_csv(merged_path, index=False, float_format="%.4f")
    print(f"\n  Saved merged data: {merged_path}")
    
    # Prepare two "views" for ranking
    # View 1: LLM-as-judge scores (score_document, score_reference, score_reference_document)
    # View 2: Code similarity metrics (codebleu, rouge, etc.)
    
    df = normalize_cols(merged_df)
    model_col = detect_model_col(df)
    numeric_cols, score_like = get_numeric_and_score_like(df)
    
    # Split into two logical groups
    llm_judge_cols = [c for c in df.columns if 'score_' in c.lower()]
    code_sim_cols = [c for c in score_like if c not in llm_judge_cols]
    
    zparts = []
    per_file_info = {}
    
    # Rank by LLM-judge scores (primary: score_reference_document)
    if llm_judge_cols:
        primary = 'score_reference_document' if 'score_reference_document' in df.columns else llm_judge_cols[0]
        rank_df = rank_one_file(df, model_col, primary, False, llm_judge_cols)
        out_csv = OUT_DIR / "ranking_by_llm_judge.csv"
        rank_df.to_csv(out_csv, index=False, float_format="%.2f")
        per_file_info["llm_judge"] = {"metric": primary, "csv": out_csv}
        z = zscore(rank_df["RankMetric"])
        zparts.append(pd.DataFrame({"model": rank_df[model_col].values, "z_llm_judge": z.values}))
        print(f"  Ranked by LLM-judge ({primary}): {out_csv}")
    
    # Rank by code similarity (primary: codebleu or mean)
    if code_sim_cols:
        primary = 'codebleu' if 'codebleu' in df.columns else "__mean_scores__"
        use_mean = primary == "__mean_scores__"
        rank_df = rank_one_file(df, model_col, primary, use_mean, code_sim_cols)
        out_csv = OUT_DIR / "ranking_by_code_similarity.csv"
        rank_df.to_csv(out_csv, index=False, float_format="%.4f")
        per_file_info["code_sim"] = {"metric": primary, "csv": out_csv}
        z = zscore(rank_df["RankMetric"])
        zparts.append(pd.DataFrame({"model": rank_df[model_col].values, "z_code_sim": z.values}))
        print(f"  Ranked by code similarity ({primary}): {out_csv}")
    
    # Build consensus
    if not zparts:
        raise SystemExit("[ERROR] No rankable data found.")
    
    cons = zparts[0]
    for part in zparts[1:]:
        cons = cons.merge(part, on="model", how="outer")
    
    zcols = [c for c in cons.columns if c.startswith("z_")]
    cons["ConsensusZ"] = cons[zcols].mean(axis=1, skipna=True)
    cons["ConsensusScore"] = (minmax01(cons["ConsensusZ"]) * 100.0).round(2)
    cons = cons.sort_values("ConsensusZ", ascending=False).reset_index(drop=True)
    cons["Rank"] = np.arange(1, len(cons) + 1)
    cons = cons[["Rank", "model", "ConsensusScore"] + zcols]
    
    cons_path = OUT_DIR / "consensus_llm_rankings.csv"
    cons.to_csv(cons_path, index=False, float_format="%.2f")
    print(f"  Consensus ranking: {cons_path}")
    
    # All metrics with rank (aggregated per model)
    numeric_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
    metrics_agg = df.groupby(model_col)[numeric_cols].mean().reset_index()
    metrics_agg = metrics_agg.rename(columns={model_col: "model"})
    
    all_with_rank = cons[["Rank", "model", "ConsensusScore"]].merge(metrics_agg, on="model", how="left")
    all_with_rank = all_with_rank.sort_values("Rank").reset_index(drop=True)
    
    all_path = OUT_DIR / "llm_all_metrics_with_rank.csv"
    all_with_rank.to_csv(all_path, index=False, float_format="%.2f")
    print(f"  All metrics with rank: {all_path}")
    
    return {
        "consensus": cons,
        "all_metrics": all_with_rank,
        "per_file_info": per_file_info
    }


# =============================================================================
# STEP 5: Generate plot
# =============================================================================
def make_plot(cons: pd.DataFrame):
    if not MAKE_PLOT or not MATPLOTLIB_OK:
        return
    
    print("\n" + "=" * 60)
    print("Step 3: Generating plot...")
    print("=" * 60)
    
    top10 = cons.head(10)
    
    plt.figure(figsize=(10, 5))
    colors = plt.cm.RdYlGn(np.linspace(0.8, 0.3, len(top10)))
    bars = plt.bar(top10["model"].astype(str), top10["ConsensusScore"], color=colors)
    
    # Add value labels
    for bar, score in zip(bars, top10["ConsensusScore"]):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, 
                f'{score:.1f}', ha='center', fontsize=9, fontweight='bold')
    
    plt.xticks(rotation=45, ha="right")
    plt.title("ChronoBench: Top 10 LLMs — Consensus Score", fontsize=14, fontweight='bold')
    plt.xlabel("Model")
    plt.ylabel("Consensus Score (0–100)")
    plt.ylim(0, 105)
    plt.tight_layout()
    
    plot_path = OUT_DIR / "consensus_llm_rankings_top10.png"
    plt.savefig(plot_path, dpi=160)
    plt.close()
    print(f"  Plot saved: {plot_path}")


# =============================================================================
# MAIN
# =============================================================================
def main():
    print("\n" + "=" * 60)
    print("  ChronoBench LLM Ranking Pipeline")
    print("=" * 60)
    
    # Check inputs exist
    for p in [COMBINED_SCORES, EVAL_RESULTS]:
        if not p.exists():
            raise SystemExit(f"[ERROR] Missing input: {p}")
    
    # Step 1: Merge files
    merged_df = merge_evaluation_files()
    
    # Step 2: Build rankings
    results = build_rankings(merged_df)
    
    # Step 3: Generate plot
    make_plot(results["consensus"])
    
    # Summary
    print("\n" + "=" * 60)
    print("  DONE! Output files:")
    print("=" * 60)
    for f in sorted(OUT_DIR.glob("*.csv")):
        print(f"  - {f.name}")
    for f in sorted(OUT_DIR.glob("*.png")):
        print(f"  - {f.name}")
    
    # Print top 10
    print("\n" + "=" * 60)
    print("  Top 10 LLMs:")
    print("=" * 60)
    top10 = results["consensus"].head(10)[["Rank", "model", "ConsensusScore"]]
    print(top10.to_string(index=False))


if __name__ == "__main__":
    main()
