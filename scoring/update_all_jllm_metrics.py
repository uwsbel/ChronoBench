#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Update all JLLM metrics in out_diff_models subdirectories.

This script processes all subdirectories in scoring/out_diff_models/,
merging JLLM-specific scores with common metrics to create complete
llm_all_metrics_with_rank.csv files.
"""

from pathlib import Path
import re
import numpy as np
import pandas as pd
import sys
from typing import List, Dict, Tuple

# Base directory containing all JLLM outputs
BASE_DIR = Path("/home/hongyu/Documents/SimBench/scoring/out_diff_models")

def normalize_cols(df: pd.DataFrame) -> pd.DataFrame:
    """Normalize column names by stripping whitespace."""
    df = df.copy()
    df.columns = [c.strip() for c in df.columns]
    return df

def detect_model_col(df: pd.DataFrame) -> str:
    """Detect the model column name in a dataframe."""
    lower = {c.lower(): c for c in df.columns}
    for key in ["model", "test model", "llm", "model_name"]:
        if key in lower:
            return lower[key]
    return df.columns[0]

def get_numeric_and_score_like(df: pd.DataFrame) -> Tuple[List[str], List[str]]:
    """Get numeric columns and score-like columns for ranking."""
    numeric_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
    
    # Exclude flag-like patterns from score metrics
    flag_like_patterns = [r"compile", r"pass", r"success", r"fail", r"error", r"warning"]
    flag_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in flag_like_patterns)]
    
    # Include score-like patterns
    score_like_patterns = [
        r"score", r"bleu", r"rouge", r"codebleu",
        r"exact", r"\bf1\b", r"\bem\b", r"accuracy", r"\bacc\b",
        r"mcc", r"pearson", r"spearman", r"reference", r"document",
        r"ngram", r"syntax", r"dataflow"
    ]
    score_like = [c for c in numeric_cols if any(re.search(p, c, flags=re.I) for p in score_like_patterns)]
    score_like = [c for c in score_like if c not in flag_like]
    
    return numeric_cols, score_like

def pick_primary_metric(df: pd.DataFrame) -> Tuple[str, bool]:
    """Pick the primary metric for ranking."""
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
    """Calculate z-scores for a series."""
    s = pd.to_numeric(series, errors="coerce")
    mu = s.mean()
    sd = s.std(ddof=0)
    if sd == 0 or np.isnan(sd):
        return pd.Series(np.zeros(len(s)), index=s.index)
    return (s - mu) / sd

def minmax01(series: pd.Series) -> pd.Series:
    """Normalize series to 0-1 range."""
    s = pd.to_numeric(series, errors="coerce")
    mn, mx = s.min(), s.max()
    if mx == mn:
        return pd.Series(np.ones(len(s)) * 0.5, index=s.index)
    return (s - mn) / (mx - mn)

def process_jllm_directory(jllm_dir: Path) -> bool:
    """
    Process a single JLLM directory to update its llm_all_metrics_with_rank.csv.
    
    Returns True if successful, False otherwise.
    """
    print(f"\nProcessing: {jllm_dir.name}")
    
    # Find the JLLM-specific score file
    score_files = list(jllm_dir.glob("combined_evaluation_scores_*.csv"))
    if not score_files:
        # Try the generic name
        generic_score = jllm_dir / "combined_evaluation_scores.csv"
        if generic_score.exists():
            score_files = [generic_score]
        else:
            print(f"  Warning: No combined_evaluation_scores file found in {jllm_dir.name}")
            return False
    
    score_file = score_files[0]
    eval_file = jllm_dir / "evaluation_results.csv"
    
    if not eval_file.exists():
        print(f"  Warning: evaluation_results.csv not found in {jllm_dir.name}")
        return False
    
    try:
        # Load and normalize dataframes
        score_df = normalize_cols(pd.read_csv(score_file))
        eval_df = normalize_cols(pd.read_csv(eval_file))
        
        # Detect model columns
        score_model_col = detect_model_col(score_df)
        eval_model_col = detect_model_col(eval_df)
        
        # Standardize model column names
        score_df = score_df.rename(columns={score_model_col: "model"})
        eval_df = eval_df.rename(columns={eval_model_col: "model"})
        
        # Handle 'FAILED' values in score_df by converting them to NaN
        for col in score_df.columns:
            if col != "model":
                # Replace 'FAILED' strings with NaN for numeric processing
                score_df[col] = pd.to_numeric(score_df[col], errors='coerce')
        
        # Aggregate metrics by model (average across systems and rounds)
        # For score_df
        score_numeric = [c for c in score_df.columns if c != "model"]
        score_agg = score_df.groupby("model")[score_numeric].mean().reset_index()
        
        # Log models with many failed evaluations
        failed_counts = score_df.groupby("model")[score_numeric].apply(lambda x: x.isna().sum().sum())
        for model, count in failed_counts.items():
            if count > len(score_numeric) * 0.5:  # More than 50% failed
                print(f"  ⚠️ Model '{model}' has {count} failed evaluations")
        
        # For eval_df
        eval_numeric = [c for c in eval_df.columns if pd.api.types.is_numeric_dtype(eval_df[c])]
        eval_agg = eval_df.groupby("model")[eval_numeric].mean().reset_index()
        
        # Merge the dataframes using outer join to keep all models
        merged_df = pd.merge(score_agg, eval_agg, on="model", how="outer")
        
        # Calculate ranking based on available metrics
        numeric_cols, score_like = get_numeric_and_score_like(merged_df)
        primary_metric, use_mean = pick_primary_metric(merged_df)
        
        if use_mean:
            # Use mean of score-like columns for models without primary metric
            if score_like:
                merged_df["__rank_metric__"] = merged_df[score_like].mean(axis=1, skipna=True)
            else:
                merged_df["__rank_metric__"] = merged_df[numeric_cols].mean(axis=1, skipna=True)
        else:
            merged_df["__rank_metric__"] = merged_df[primary_metric]
        
        # Calculate consensus score and rank
        # Handle models with all NaN scores
        z_scores = zscore(merged_df["__rank_metric__"])
        consensus_scores = (minmax01(z_scores) * 100.0).round(2)
        
        # Set ConsensusScore to 0 for models with no valid scores
        consensus_scores[merged_df["__rank_metric__"].isna()] = 0.0
        merged_df["ConsensusScore"] = consensus_scores
        
        # Log models with zero consensus scores
        zero_consensus = merged_df[merged_df["ConsensusScore"] == 0.0]["model"].tolist()
        if zero_consensus:
            print(f"  ⚠️ Models with zero consensus score: {', '.join(zero_consensus)}")
        
        # Sort by consensus score and assign ranks
        merged_df = merged_df.sort_values("ConsensusScore", ascending=False).reset_index(drop=True)
        merged_df["Rank"] = np.arange(1, len(merged_df) + 1)
        
        # Remove temporary ranking column
        merged_df = merged_df.drop(columns=["__rank_metric__"], errors="ignore")
        
        # Reorder columns to put Rank and model first
        cols = ["Rank", "model", "ConsensusScore"]
        remaining_cols = [c for c in merged_df.columns if c not in cols]
        merged_df = merged_df[cols + remaining_cols]
        
        # Save the updated metrics file
        output_file = jllm_dir / "llm_all_metrics_with_rank.csv"
        merged_df.to_csv(output_file, index=False, float_format="%.2f")
        
        print(f"  ✓ Updated {output_file.name} with {len(merged_df)} models")
        return True
        
    except Exception as e:
        print(f"  Error processing {jllm_dir.name}: {e}")
        return False

def main():
    """Main function to process all JLLM directories."""
    print("=" * 60)
    print("Updating JLLM Metrics in out_diff_models")
    print("=" * 60)
    
    # Find all out_* subdirectories
    jllm_dirs = sorted([d for d in BASE_DIR.iterdir() if d.is_dir() and d.name.startswith("out_")])
    
    if not jllm_dirs:
        print("No out_* directories found in", BASE_DIR)
        return 1
    
    print(f"Found {len(jllm_dirs)} JLLM directories to process")
    
    # Process each directory
    successful = 0
    failed = 0
    
    for jllm_dir in jllm_dirs:
        if process_jllm_directory(jllm_dir):
            successful += 1
        else:
            failed += 1
    
    # Summary
    print("\n" + "=" * 60)
    print("Processing Complete")
    print(f"  Successful: {successful}")
    print(f"  Failed: {failed}")
    print("=" * 60)
    
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())