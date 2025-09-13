#!/usr/bin/env python3
"""
Comprehensive LLM Ranking System for SimBench
Implements z-score based ranking methodology from rank_llm.py
Fixes scale detection issues and handles all 24 student LLMs properly
"""

import os
import re
import pandas as pd
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import warnings
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Paths
BASE_DIR = Path("/home/hongyu/Documents/andy_simbench/SimBench")
OUTPUT_DIR = BASE_DIR / "output_llms"
STAT_DIR = BASE_DIR / "statistic"
STATISTICS_DIR = BASE_DIR / "statistics"  # Alternative location

# The 24 target models
TARGET_MODELS = [
    "gemma-3-27b-it",
    "llama-3.1-405b-instruct",
    "mistral-large-latest",
    "llama4_maverick",
    "llama-3.3-70b-instruct",
    "qwen3-235b-a22b",
    "llama4_scout",
    "codestral-22b-instruct-v0.1",
    "mixtral-8x22b-instruct-v0.1",
    "deepseek-r1",
    "deepseek-r1-32b",
    "llama-3.1-70b-instruct",
    "gemma-2-27b-it",
    "nemotron-4-340b-instruct",
    "llama-3.1-8b-instruct",
    "mistral-nemo-12b-instruct",
    "deepseek-r1-8b",
    "gemma-2-2b-it",
    "mixtral-8x7b-instruct-v0.1",
    "gemma-3-1b-it",
    "mamba-codestral-7b-v0.1",
    "phi-3-mini-128k-instruct",
    "gemma-2-9b-it",
    "phi-3-medium-128k-instruct"
]

# Systems to evaluate
SYSTEMS = [
    "art", "beam", "buckling", "cable", "camera",
    "citybus", "curiosity", "feda", "gator", "gear",
    "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113",
    "man", "mass_spring_damper", "particles", "pendulum",
    "rigid_highway", "rigid_multipatches", "rotor", "scm",
    "scm_hill", "sedan", "sensros", "slider_crank",
    "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"
]

class RankingSystem:
    """Main ranking system implementing z-score methodology"""
    
    def __init__(self):
        self.warnings_log = []
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def extract_score(self, text: str) -> Optional[float]:
        """Extract numeric score from evaluation text"""
        if not text or text.startswith("ERROR:"):
            return None
        matches = re.findall(r"\[\[(\d+(?:\.\d+)?)\]\]", text)
        if matches:
            return float(matches[-1])
        return None
    
    def detect_and_fix_scale(self, scores: List[float], model_name: str, score_type: str) -> List[float]:
        """
        Detect scale and fix safely
        Returns scores in 0-100 scale with proper validation
        """
        if not scores:
            return scores
        
        scores = [s for s in scores if s is not None]
        if not scores:
            return []
        
        max_score = max(scores)
        min_score = min(scores)
        avg_score = np.mean(scores)
        
        # Special handling for gemma-3-27b-it (already in 0-100)
        if model_name == "gemma-3-27b-it":
            # Just cap at 100 for safety
            fixed = [min(s, 100) for s in scores]
            if max_score > 100:
                self.warnings_log.append(f"WARNING: {model_name} {score_type} had scores > 100, capped at 100")
            return fixed
        
        # Detect scale
        if max_score > 100:
            # Error case: scores exceed 100 (like phi-3-medium bug)
            self.warnings_log.append(
                f"ERROR: {model_name} {score_type} has scores > 100 (max={max_score:.2f}). "
                f"Likely double-multiplication bug. Capping at 100."
            )
            return [min(s, 100) for s in scores]
        
        elif max_score <= 10:
            # Definitely 0-10 scale, need multiplication
            self.warnings_log.append(
                f"INFO: {model_name} {score_type} detected in 0-10 scale (max={max_score:.2f}), converting to 0-100"
            )
            return [s * 10 for s in scores]
        
        elif avg_score < 10 and max_score < 20:
            # Likely 0-10 scale (low average but one outlier below 20)
            self.warnings_log.append(
                f"INFO: {model_name} {score_type} likely in 0-10 scale (avg={avg_score:.2f}, max={max_score:.2f}), converting to 0-100"
            )
            return [s * 10 for s in scores]
        
        else:
            # Already in 0-100 scale
            return scores
    
    def collect_judge_scores(self) -> pd.DataFrame:
        """Collect judge scores from averaged CSV files"""
        logging.info("Collecting averaged judge scores...")
        
        # First try to read from combined averaged CSV
        import glob
        pattern = str(OUTPUT_DIR / "combined_evaluation_scores_averaged_*.csv")
        avg_files = sorted(glob.glob(pattern))
        
        if avg_files:
            # Use the most recent averaged file
            latest_avg_file = avg_files[-1]
            logging.info(f"Reading averaged scores from: {latest_avg_file}")
            
            try:
                df = pd.read_csv(latest_avg_file)
                
                # Aggregate scores by model (average across all systems and rounds)
                result = df.groupby('Model').agg({
                    'Avg Score Document': 'mean',
                    'Avg Score Reference': 'mean',
                    'Avg Score Ref+Doc': 'mean'
                }).reset_index()
                
                result.columns = ['model', 'score_document', 'score_reference', 'score_ref_doc']
                
                # Filter to only target models
                result = result[result['model'].isin(TARGET_MODELS)]
                
                # Add missing models with NaN
                for model in TARGET_MODELS:
                    if model not in result['model'].values:
                        logging.warning(f"Model {model} not found in averaged scores")
                        new_row = pd.DataFrame([{
                            'model': model,
                            'score_document': np.nan,
                            'score_reference': np.nan,
                            'score_ref_doc': np.nan
                        }])
                        result = pd.concat([result, new_row], ignore_index=True)
                
                return result
                
            except Exception as e:
                logging.error(f"Error reading averaged CSV: {e}")
                logging.info("Falling back to single judge scores...")
        
        # Fall back to original single judge score collection
        logging.info("No averaged score files found, using single judge scores...")
        
        records = []
        
        for model_name in TARGET_MODELS:
            model_dir = OUTPUT_DIR / model_name
            
            if not model_dir.exists():
                logging.warning(f"Model directory not found: {model_dir}")
                # Add zero scores for missing models
                records.append({
                    'model': model_name,
                    'score_document': 0.0,
                    'score_reference': 0.0,
                    'score_ref_doc': 0.0
                })
                continue
            
            # Collect all scores
            doc_scores = []
            ref_scores = []
            refdoc_scores = []
            
            for system in SYSTEMS:
                sys_dir = model_dir / system
                if not sys_dir.exists():
                    continue
                
                for round_name in ["first", "second", "third"]:
                    # Try multiple patterns for score files
                    patterns = [
                        f"{round_name}_score_document.txt",
                        f"{round_name}_score_reference.txt",
                        f"{round_name}_score_reference_document.txt"
                    ]
                    
                    doc_file = sys_dir / patterns[0]
                    ref_file = sys_dir / patterns[1]
                    refdoc_file = sys_dir / patterns[2]
                    
                    if doc_file.exists():
                        try:
                            with open(doc_file, 'r') as f:
                                score = self.extract_score(f.read())
                                if score is not None:
                                    doc_scores.append(score)
                        except Exception as e:
                            logging.warning(f"Error reading {doc_file}: {e}")
                    
                    if ref_file.exists():
                        try:
                            with open(ref_file, 'r') as f:
                                score = self.extract_score(f.read())
                                if score is not None:
                                    ref_scores.append(score)
                        except Exception as e:
                            logging.warning(f"Error reading {ref_file}: {e}")
                    
                    if refdoc_file.exists():
                        try:
                            with open(refdoc_file, 'r') as f:
                                score = self.extract_score(f.read())
                                if score is not None:
                                    refdoc_scores.append(score)
                        except Exception as e:
                            logging.warning(f"Error reading {refdoc_file}: {e}")
            
            # Fix scales and calculate averages
            doc_scores_fixed = self.detect_and_fix_scale(doc_scores, model_name, "document")
            ref_scores_fixed = self.detect_and_fix_scale(ref_scores, model_name, "reference")
            refdoc_scores_fixed = self.detect_and_fix_scale(refdoc_scores, model_name, "ref_doc")
            
            avg_doc = np.mean(doc_scores_fixed) if doc_scores_fixed else 0.0
            avg_ref = np.mean(ref_scores_fixed) if ref_scores_fixed else 0.0
            avg_refdoc = np.mean(refdoc_scores_fixed) if refdoc_scores_fixed else 0.0
            
            records.append({
                'model': model_name,
                'score_document': avg_doc,
                'score_reference': avg_ref,
                'score_ref_doc': avg_refdoc
            })
            
            logging.info(f"  {model_name}: doc={avg_doc:.2f}, ref={avg_ref:.2f}, refdoc={avg_refdoc:.2f}")
        
        return pd.DataFrame(records)
    
    def collect_similarity_metrics(self) -> pd.DataFrame:
        """Collect similarity metrics from evaluation_results_1.csv"""
        logging.info("Collecting similarity metrics...")
        
        # Try both possible locations
        for stats_dir in [STAT_DIR, STATISTICS_DIR]:
            eval_file = stats_dir / "evaluation_results_1.csv"  # ONLY use _1 file
            if eval_file.exists():
                try:
                    df = pd.read_csv(eval_file)
                    
                    # Get ALL numeric columns from the CSV
                    numeric_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
                    
                    # Aggregate ALL numeric columns by model
                    agg_dict = {col: 'mean' for col in numeric_cols}
                    metrics = df.groupby('model').agg(agg_dict).reset_index()
                    
                    # Filter to only target models
                    metrics = metrics[metrics['model'].isin(TARGET_MODELS)]
                    
                    # Add missing models with NaN and log errors
                    for model in TARGET_MODELS:
                        if model not in metrics['model'].values:
                            logging.error(f"ERROR: Model '{model}' not found in evaluation_results_1.csv!")
                            new_row = {
                                'model': model,
                                'codebleu': np.nan,
                                'ngram_match_score': np.nan,
                                'weighted_ngram_match_score': np.nan,
                                'syntax_match_score': np.nan,
                                'dataflow_match_score': np.nan,
                                'rouge1': np.nan,
                                'rouge2': np.nan,
                                'rougeL': np.nan,
                                'rougeLsum': np.nan
                            }
                            metrics = pd.concat([metrics, pd.DataFrame([new_row])], ignore_index=True)
                    
                    return metrics
                    
                except Exception as e:
                    logging.error(f"Error reading {eval_file}: {e}")
        
        # Return empty dataframe with all models if no file found
        logging.error("ERROR: No evaluation_results_1.csv found! All metrics will be NaN")
        return pd.DataFrame({
            'model': TARGET_MODELS,
            'codebleu': [np.nan] * len(TARGET_MODELS),
            'ngram_match_score': [np.nan] * len(TARGET_MODELS),
            'weighted_ngram_match_score': [np.nan] * len(TARGET_MODELS),
            'syntax_match_score': [np.nan] * len(TARGET_MODELS),
            'dataflow_match_score': [np.nan] * len(TARGET_MODELS),
            'rouge1': [np.nan] * len(TARGET_MODELS),
            'rouge2': [np.nan] * len(TARGET_MODELS),
            'rougeL': [np.nan] * len(TARGET_MODELS),
            'rougeLsum': [np.nan] * len(TARGET_MODELS)
        })
    
    def calculate_zscore(self, series: pd.Series) -> pd.Series:
        """Calculate z-scores for a series"""
        s = pd.to_numeric(series, errors="coerce")
        mu = s.mean()
        sd = s.std(ddof=0)
        if sd == 0 or np.isnan(sd):
            return pd.Series(np.zeros(len(s)), index=s.index)
        return (s - mu) / sd
    
    def minmax_scale(self, series: pd.Series) -> pd.Series:
        """Min-max scale to 0-1 range"""
        s = pd.to_numeric(series, errors="coerce")
        mn, mx = s.min(), s.max()
        if mx == mn:
            return pd.Series(np.ones(len(s)) * 0.5, index=s.index)
        return (s - mn) / (mx - mn)
    
    def calculate_consensus_rankings(self, judge_df: pd.DataFrame, similarity_df: pd.DataFrame) -> pd.DataFrame:
        """
        Calculate consensus rankings using z-score methodology
        """
        # Merge dataframes
        df = pd.merge(judge_df, similarity_df, on='model', how='outer')
        
        # Filter to only include the 24 target models
        df = df[df['model'].isin(TARGET_MODELS)]
        
        # Fill any NaN values with 0
        df = df.fillna(0)
        
        # Calculate z-scores for each metric
        z_scores = pd.DataFrame(index=df.index)
        z_scores['model'] = df['model']
        
        # Judge metrics z-scores (already in 0-100)
        judge_metrics = ['score_document', 'score_reference', 'score_ref_doc']
        for metric in judge_metrics:
            if metric in df.columns:
                z_scores[f'z_{metric}'] = self.calculate_zscore(df[metric])
        
        # Similarity metrics z-scores (need to scale to 0-100 first)
        similarity_metrics = ['codebleu', 'rouge1', 'rougeL']
        for metric in similarity_metrics:
            if metric in df.columns:
                # Convert 0-1 scale to 0-100 for consistency
                scaled = df[metric] * 100
                z_scores[f'z_{metric}'] = self.calculate_zscore(scaled)
        
        # Calculate weighted consensus z-score
        # 70% weight to judge scores, 30% to similarity metrics
        judge_z_cols = [col for col in z_scores.columns if col.startswith('z_score_')]
        similarity_z_cols = [col for col in z_scores.columns if col.startswith('z_') and 'score_' not in col]
        
        if judge_z_cols:
            judge_avg_z = z_scores[judge_z_cols].mean(axis=1, skipna=True)
        else:
            judge_avg_z = 0
        
        if similarity_z_cols:
            similarity_avg_z = z_scores[similarity_z_cols].mean(axis=1, skipna=True)
        else:
            similarity_avg_z = 0
        
        # Weighted combination
        z_scores['consensus_z'] = (judge_avg_z * 0.7) + (similarity_avg_z * 0.3)
        
        # Apply min-max scaling to get 0-100 consensus score
        df['consensus_score'] = (self.minmax_scale(z_scores['consensus_z']) * 100).round(2)
        
        # Sort and rank
        df = df.sort_values('consensus_score', ascending=False)
        df['rank'] = range(1, len(df) + 1)
        
        # Add z-score columns for transparency
        for col in z_scores.columns:
            if col.startswith('z_'):
                df[col] = z_scores[col].round(3)
        
        return df
    
    def generate_outputs(self, rankings_df: pd.DataFrame):
        """Generate all output files"""
        output_base = OUTPUT_DIR / f"llm_rankings_{self.timestamp}"
        
        # 1. Main rankings CSV
        main_cols = ['rank', 'model', 'consensus_score', 'score_document', 
                    'score_reference', 'score_ref_doc', 'codebleu', 
                    'ngram_match_score', 'weighted_ngram_match_score', 
                    'syntax_match_score', 'dataflow_match_score',
                    'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
        main_df = rankings_df[main_cols].round(2)
        main_file = f"{output_base}.csv"
        main_df.to_csv(main_file, index=False)
        logging.info(f"Main rankings saved to: {main_file}")
        
        # 2. Detailed metrics CSV (includes z-scores)
        detailed_file = f"{output_base}_detailed.csv"
        rankings_df.round(3).to_csv(detailed_file, index=False)
        logging.info(f"Detailed rankings saved to: {detailed_file}")
        
        # 3. Methodology report
        method_file = f"{output_base}_methodology.txt"
        with open(method_file, 'w') as f:
            f.write("LLM RANKING METHODOLOGY\n")
            f.write("=" * 50 + "\n\n")
            f.write("This ranking system uses z-score based consensus scoring:\n\n")
            f.write("1. Data Collection:\n")
            f.write("   - Judge scores from output_llms/*/score files\n")
            f.write("   - Similarity metrics from evaluation_results.csv\n\n")
            f.write("2. Scale Normalization:\n")
            f.write("   - Detect if scores are in 0-10 or 0-100 scale\n")
            f.write("   - Convert all to 0-100 scale\n")
            f.write("   - Cap any scores > 100 (error correction)\n\n")
            f.write("3. Z-Score Calculation:\n")
            f.write("   - Calculate z-scores for each metric\n")
            f.write("   - Z-score = (value - mean) / std_dev\n\n")
            f.write("4. Consensus Score:\n")
            f.write("   - Judge metrics weight: 70%\n")
            f.write("   - Similarity metrics weight: 30%\n")
            f.write("   - Final score: min-max scaled to 0-100\n\n")
            f.write("5. Special Handling:\n")
            f.write("   - gemma-3-27b-it: No scale multiplication (already 0-100)\n")
            f.write("   - phi-3-medium: Cap scores at 100 (fix double-multiplication)\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        logging.info(f"Methodology saved to: {method_file}")
        
        # 4. Warnings log
        if self.warnings_log:
            warnings_file = f"{output_base}_warnings.log"
            with open(warnings_file, 'w') as f:
                f.write("SCALE DETECTION AND CORRECTION LOG\n")
                f.write("=" * 50 + "\n\n")
                for warning in self.warnings_log:
                    f.write(warning + "\n")
            logging.info(f"Warnings saved to: {warnings_file}")
        
        return main_file
    
    def run(self):
        """Main execution method"""
        print("=" * 80)
        print("LLM RANKING SYSTEM - Z-SCORE METHODOLOGY")
        print("=" * 80)
        
        # Collect data
        judge_df = self.collect_judge_scores()
        similarity_df = self.collect_similarity_metrics()
        
        # Calculate rankings
        rankings_df = self.calculate_consensus_rankings(judge_df, similarity_df)
        
        # Generate outputs
        main_file = self.generate_outputs(rankings_df)
        
        # Display results
        print("\n" + "=" * 80)
        print("TOP 10 RANKINGS")
        print("=" * 80)
        print(rankings_df.head(10)[['rank', 'model', 'consensus_score', 
                                    'score_document', 'score_reference', 'score_ref_doc']].to_string(index=False))
        
        # Show specific models of interest
        print("\n" + "=" * 80)
        print("MODELS OF INTEREST")
        print("=" * 80)
        
        for model in ['gemma-3-27b-it', 'phi-3-medium-128k-instruct']:
            model_data = rankings_df[rankings_df['model'] == model]
            if not model_data.empty:
                rank = model_data.iloc[0]['rank']
                score = model_data.iloc[0]['consensus_score']
                print(f"{model}: Rank #{rank}, Score {score:.2f}")
        
        print("\n" + "=" * 80)
        print("SUMMARY")
        print("=" * 80)
        print(f"✓ Ranked {len(rankings_df)} models")
        print(f"✓ Main rankings saved to: {main_file}")
        print(f"✓ Warnings logged: {len(self.warnings_log)} issues found")
        
        if self.warnings_log:
            print("\nKey warnings:")
            for warning in self.warnings_log[:3]:  # Show first 3 warnings
                if "ERROR" in warning or "gemma-3-27b-it" in warning or "phi-3-medium" in warning:
                    print(f"  - {warning[:100]}...")
        
        return rankings_df


if __name__ == "__main__":
    ranker = RankingSystem()
    rankings = ranker.run()