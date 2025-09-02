#!/usr/bin/env python3
"""
Consistency Check Script for SimBench Pipeline
This script checks for common consistency issues and can fix them automatically.
"""

import os
import sys
import pandas as pd
from pathlib import Path
from typing import List, Set
import argparse

# Import the auto-discovery function
from auto_discover_models import discover_models, export_model_list
from validate_pipeline import PipelineValidator

# Configuration
DATA_ROOT = Path("/home/hongyu/Documents/SimBench")
OUTPUT_DIR = DATA_ROOT / "output_llms"
STATISTIC_DIR = DATA_ROOT / "statistic"
SCORING_DIR = DATA_ROOT / "scoring"

JLLM_JUDGES = ["gpt-4o-mini", "gpt-4.1-mini", "gpt-4.1-nano"]


def check_missing_metrics():
    """Check for models with missing similarity metrics."""
    print("\n📊 Checking for missing similarity metrics...")
    
    # Get all models with outputs
    all_models = set(discover_models())
    
    # Load current metrics
    metrics_file = STATISTIC_DIR / "evaluation_results.csv"
    if not metrics_file.exists():
        print(f"❌ Metrics file does not exist: {metrics_file}")
        return all_models
    
    df = pd.read_csv(metrics_file)
    models_with_metrics = set(df['model'].unique())
    
    # Find missing
    missing = all_models - models_with_metrics
    
    if missing:
        print(f"⚠️  Found {len(missing)} models without metrics:")
        for model in sorted(missing):
            print(f"   - {model}")
    else:
        print("✅ All models have similarity metrics")
    
    return missing


def check_jllm_consistency():
    """Check JLLM rankings for missing models or empty metrics."""
    print("\n🏆 Checking JLLM rankings consistency...")
    
    issues = []
    all_models = set(discover_models())
    
    for judge in JLLM_JUDGES:
        judge_dir = DATA_ROOT / f"output_llms_{judge.replace('.', '-')}"
        ranking_file = judge_dir / "all_scores_ranked.csv"
        
        if not ranking_file.exists():
            print(f"⚠️  JLLM ranking file missing: {ranking_file}")
            continue
        
        df = pd.read_csv(ranking_file)
        
        # Check for models with empty metrics
        for _, row in df.iterrows():
            model = row['model'].strip()
            if pd.isna(row.get('codebleu', None)) or row.get('codebleu', '') == '':
                issues.append((judge, model))
    
    if issues:
        print(f"❌ Found {len(issues)} models with empty metrics in JLLM rankings:")
        for judge, model in issues[:10]:  # Show first 10
            print(f"   - {model} in {judge}")
        if len(issues) > 10:
            print(f"   ... and {len(issues) - 10} more")
    else:
        print("✅ All JLLM rankings have complete metrics")
    
    return issues


def fix_missing_metrics(models: Set[str], dry_run: bool = True):
    """Compute missing similarity metrics for specified models."""
    if not models:
        print("\n✅ No missing metrics to fix")
        return
    
    print(f"\n🔧 {'Would compute' if dry_run else 'Computing'} metrics for {len(models)} models...")
    
    if dry_run:
        print("   Run with --fix to actually compute metrics")
        return
    
    # Import scoring module
    sys.path.insert(0, str(SCORING_DIR))
    os.chdir(SCORING_DIR)
    
    # Run p_sim_score.py for missing models
    os.system(f"python p_sim_score.py")
    
    print("✅ Metrics computation complete")


def regenerate_jllm_rankings(dry_run: bool = True):
    """Regenerate JLLM rankings with updated metrics."""
    print(f"\n🔄 {'Would regenerate' if dry_run else 'Regenerating'} JLLM rankings...")
    
    if dry_run:
        print("   Run with --fix to actually regenerate rankings")
        return
    
    os.chdir(SCORING_DIR)
    
    # Run the JLLM ranking generation script
    for judge in JLLM_JUDGES:
        print(f"   Generating rankings for {judge}...")
        os.system(f"python generate_jllm_all_scores_ranked.py")
    
    print("✅ JLLM rankings regenerated")


def update_model_list():
    """Update the model list with newly discovered models."""
    print("\n🔄 Updating model list...")
    
    # Run auto-discovery
    os.chdir(SCORING_DIR)
    result = export_model_list()
    
    print(f"✅ Model list updated with {result['total_models']} models")


def main():
    parser = argparse.ArgumentParser(description="Check and fix SimBench pipeline consistency")
    parser.add_argument('--fix', action='store_true', 
                       help='Actually fix issues (default is dry-run)')
    parser.add_argument('--validate-only', action='store_true',
                       help='Only run validation, no fixes')
    args = parser.parse_args()
    
    print("=" * 60)
    print("SimBench Pipeline Consistency Check")
    print("=" * 60)
    
    # Always update model discovery first
    update_model_list()
    
    # Check for issues
    missing_metrics = check_missing_metrics()
    jllm_issues = check_jllm_consistency()
    
    if args.validate_only:
        # Run full validation
        print("\n" + "=" * 60)
        print("Running Full Validation")
        print("=" * 60)
        validator = PipelineValidator()
        success = validator.validate()
        
        if success:
            print("\n✅ Validation passed")
        else:
            print("\n❌ Validation failed - see issues above")
            sys.exit(1)
    else:
        # Fix issues if requested
        if missing_metrics or jllm_issues:
            print("\n" + "=" * 60)
            print("Suggested Fixes")
            print("=" * 60)
            
            if missing_metrics:
                print(f"\n1. Compute metrics for {len(missing_metrics)} models")
                fix_missing_metrics(missing_metrics, dry_run=not args.fix)
            
            if jllm_issues:
                print(f"\n2. Regenerate JLLM rankings")
                regenerate_jllm_rankings(dry_run=not args.fix)
            
            if not args.fix:
                print("\n💡 Run with --fix flag to apply these fixes automatically")
        else:
            print("\n✅ No consistency issues found!")
    
    print("\n" + "=" * 60)
    print("Check Complete")
    print("=" * 60)


if __name__ == "__main__":
    main()