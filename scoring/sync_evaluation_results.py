#!/usr/bin/env python3
"""
Sync the main evaluation_results.csv to all JLLM directories.
This ensures all directories have the complete metrics for all models.
"""

import shutil
from pathlib import Path

# Paths
BASE_DIR = Path("/home/hongyu/Documents/SimBench/scoring/out_diff_models")
SOURCE_FILE = Path("/home/hongyu/Documents/SimBench/statistic/evaluation_results.csv")

def main():
    print("=" * 60)
    print("Syncing evaluation_results.csv to all JLLM directories")
    print("=" * 60)
    
    if not SOURCE_FILE.exists():
        print(f"Error: Source file not found: {SOURCE_FILE}")
        return 1
    
    # Find all out_* directories
    jllm_dirs = sorted([d for d in BASE_DIR.iterdir() if d.is_dir() and d.name.startswith("out_")])
    
    if not jllm_dirs:
        print(f"No out_* directories found in {BASE_DIR}")
        return 1
    
    print(f"Source: {SOURCE_FILE}")
    print(f"Found {len(jllm_dirs)} JLLM directories to update\n")
    
    success_count = 0
    for jllm_dir in jllm_dirs:
        target_file = jllm_dir / "evaluation_results.csv"
        try:
            shutil.copy2(SOURCE_FILE, target_file)
            print(f"✓ Updated {jllm_dir.name}/evaluation_results.csv")
            success_count += 1
        except Exception as e:
            print(f"✗ Failed to update {jllm_dir.name}: {e}")
    
    print("\n" + "=" * 60)
    print(f"Updated {success_count}/{len(jllm_dirs)} directories")
    print("=" * 60)
    
    return 0 if success_count == len(jllm_dirs) else 1

if __name__ == "__main__":
    import sys
    sys.exit(main())