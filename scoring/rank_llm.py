#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fixed-path LLM ranking pipeline.

- Reads two fixed CSVs (set in INPUTS below).
- Ranks per file (prefers "Score Reference Document", else mean of score-like metrics).
- Builds a consensus ranking (average of per-file z-scores, scaled to 0–100).
- Writes per-file ranks, consensus ranks, wide CSV with all metrics, and a Top-10 plot.

Outputs go to OUT_DIR.
"""

from pathlib import Path
import re
import numpy as np
import pandas as pd

# ---------- FIXED PATHS (edit these if your files move) ----------
INPUTS = [
    Path("/home/hongyu/Documents/SimBench/output_llms/combined_evaluation_scores.csv"),
    Path("/home/hongyu/Documents/SimBench/statistic/evaluation_results.csv"),
]
OUT_DIR = Path("/home/hongyu/Documents/SimBench/scoring/out")   # change if you want a different output folder
MAKE_PLOT = True          # set False to skip the plot
# -----------------------------------------------------------------

# Optional plotting
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except Exception:
    MATPLOTLIB_OK = False


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
    # columns to exclude from "score-like" fallback mean
    flag_like_patterns = [r"compile", r"pass", r"success", r"fail", r"error", r"warning"]
    flag_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in flag_like_patterns)]

    score_like_patterns = [
        r"score", r"bleu", r"rouge", r"codebleu",
        r"exact", r"\bf1\b", r"\bem\b", r"accuracy", r"\bacc\b",
        r"mcc", r"pearson", r"spearman", r"reference", r"document"
    ]
    score_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in score_like_patterns)]
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

def rank_one_file(df: pd.DataFrame, model_col: str, primary_metric: str, use_mean_of_scores: bool, score_like):
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

    # Attach a few common metrics for reference if present (averaged)
    extras = []
    for pat in [
        r"Score Document", r"Score Reference", r"Score Reference Document",
        r"codebleu", r"rouge1", r"rouge2", r"rougeL", r"rougeLsum",
        r"compile", r"pass"
    ]:
        for c in df.columns:
            if re.search(pat, c, flags=re.I) and pd.api.types.is_numeric_dtype(df[c]) and c not in extras:
                extras.append(c)
    if extras:
        extra_means = df.groupby(model_col).agg({m: "mean" for m in extras}).reset_index()
        per_model = per_model.merge(extra_means, on=model_col, how="left")

    cols = ["Rank", model_col, "RankMetric"] + [c for c in per_model.columns if c not in {"Rank", model_col, "RankMetric"}]
    return per_model[cols]

def zscore(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    mu = s.mean()
    sd = s.std(ddof=0)
    if sd == 0 or np.isnan(sd):
        return pd.Series(np.zeros(len(s)), index=s.index)
    return (s - mu) / sd

def minmax01(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    mn, mx = s.min(), s.max()
    if mx == mn:
        return pd.Series(np.ones(len(s)) * 0.5, index=s.index)
    return (s - mn) / (mx - mn)

def union_all_metrics_with_rank(frames, consensus_df: pd.DataFrame) -> pd.DataFrame:
    std = []
    for df in frames:
        df = df.copy()
        mc = detect_model_col(df)
        df = df.rename(columns={mc: "model"})
        std.append(df)
    all_df = pd.concat(std, ignore_index=True)
    numeric_cols = [c for c in all_df.columns if pd.api.types.is_numeric_dtype(all_df[c])]
    metrics_agg = all_df.groupby("model")[numeric_cols].mean().reset_index()
    out = consensus_df[["Rank", "model", "ConsensusScore"]].merge(metrics_agg, on="model", how="left")
    return out.sort_values("Rank").reset_index(drop=True)

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # Load fixed inputs
    frames = {}
    for p in INPUTS:
        if not p.exists():
            raise SystemExit(f"[ERROR] Missing input: {p}")
        try:
            df = pd.read_csv(p)
        except Exception as e:
            raise SystemExit(f"[ERROR] Could not read {p}: {e}")
        frames[p.name] = normalize_cols(df)

    # Per-file ranking + z-scores
    zparts = []
    per_file_info = {}
    for name, df in frames.items():
        model_col = detect_model_col(df)
        numeric_cols, score_like = get_numeric_and_score_like(df)
        primary_metric, use_mean = pick_primary_metric(df)
        if use_mean and not score_like:
            score_like = numeric_cols

        rank_df = rank_one_file(df, model_col, primary_metric, use_mean, score_like)
        out_csv = OUT_DIR / f"{Path(name).stem}_rankings.csv"
        # --- 2 decimal places on save ---
        rank_df.to_csv(out_csv, index=False, float_format="%.2f")
        per_file_info[name] = {"primary_metric": primary_metric, "mean_based": use_mean, "csv": out_csv}

        z = zscore(rank_df["RankMetric"])
        zparts.append(pd.DataFrame({"model": rank_df[model_col].astype(str).values,
                                    f"z_{Path(name).stem}": z.values}))

    if not zparts:
        raise SystemExit("[ERROR] No rankable data found in the inputs.")

    # Consensus ranking
    cons = zparts[0]
    for part in zparts[1:]:
        cons = cons.merge(part, on="model", how="outer")
    zcols = [c for c in cons.columns if c.startswith("z_")]
    cons["ConsensusZ"] = cons[zcols].mean(axis=1, skipna=True)
    # --- round to 2 decimals ---
    cons["ConsensusScore"] = (minmax01(cons["ConsensusZ"]) * 100.0).round(2)
    cons = cons.sort_values("ConsensusZ", ascending=False).reset_index(drop=True)
    cons["Rank"] = np.arange(1, len(cons) + 1)
    cons = cons[["Rank", "model", "ConsensusScore"] + zcols]

    cons_path = OUT_DIR / "consensus_llm_rankings.csv"
    # --- 2 decimal places on save ---
    cons.to_csv(cons_path, index=False, float_format="%.2f")

    # Diff vs previous (if any)
    prev_path = OUT_DIR / "consensus_llm_rankings_prev.csv"
    if prev_path.exists():
        prev = pd.read_csv(prev_path)
        chg = cons.merge(prev[["model", "Rank", "ConsensusScore"]], on="model", how="left", suffixes=("", "_prev"))
        chg["RankChange"] = chg["Rank_prev"] - chg["Rank"]
        chg["ScoreChange"] = cons["ConsensusScore"] - chg["ConsensusScore_prev"]
        change_path = OUT_DIR / "consensus_changes.csv"
        # --- 2 decimal places on save ---
        chg.to_csv(change_path, index=False, float_format="%.2f")

    # Wide CSV with all metrics + consensus rank
    all_metrics = union_all_metrics_with_rank(list(frames.values()), cons)
    all_metrics_path = OUT_DIR / "llm_all_metrics_with_rank.csv"
    # --- 2 decimal places on save ---
    all_metrics.to_csv(all_metrics_path, index=False, float_format="%.2f")

    # Optional Top-10 plot
    if MAKE_PLOT and MATPLOTLIB_OK:
        top10 = cons.head(10)
        plt.figure(figsize=(8, 4.5))
        plt.bar(top10["model"].astype(str), top10["ConsensusScore"])
        plt.xticks(rotation=45, ha="right")
        plt.title("Top 10 LLMs — Consensus Score")
        plt.xlabel("Model")
        plt.ylabel("Consensus Score (0–100)")
        plt.tight_layout()
        plt.savefig(OUT_DIR / "consensus_llm_rankings_top10.png", dpi=160)

    # Summary
    print("\n== Outputs ==")
    print(f"Consensus: {cons_path}")
    print(f"All metrics + rank: {all_metrics_path}")
    for name, info in per_file_info.items():
        pm = info['primary_metric'] + (" [mean]" if info['mean_based'] else "")
        print(f"Per-file: {name} → {info['csv']} (by {pm})")
    if MAKE_PLOT and MATPLOTLIB_OK:
        print(f"Top-10 plot: {OUT_DIR / 'consensus_llm_rankings_top10.png'}")

if __name__ == "__main__":
    main()
