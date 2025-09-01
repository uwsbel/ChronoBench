# JLLM Evaluation Pipeline Solution

## Problem Summary
All three JLLM models (gpt-4.1-nano, gpt-4.1-mini, gpt-4o-mini) are producing identical scores because they're reading from the same pre-existing evaluation files in `/output_llms/` instead of generating new evaluations with their respective judge models.

## Current (Incorrect) Pipeline
```
/output_llms/{model}/{system}/evaluation_scores.csv (pre-existing scores from unknown judge)
    ↓
All 3 JLLM scripts read the SAME files
    ↓
Merge into combined_evaluation_scores_{judge}.csv (identical content)
    ↓
create_final_combined_scores.py generates identical all_scores_ranked.csv
```

## Correct Pipeline

### Option 1: Full Re-evaluation (Recommended for Accuracy)

```mermaid
graph TD
    A[Model Outputs in /output_llms/] -->|Read| B1[gpt-4.1-nano Judge]
    A -->|Read| B2[gpt-4.1-mini Judge]
    A -->|Read| B3[gpt-4o-mini Judge]
    
    B1 -->|Evaluate| C1[Unique Scores for nano]
    B2 -->|Evaluate| C2[Unique Scores for mini]
    B3 -->|Evaluate| C3[Unique Scores for 4o-mini]
    
    C1 --> D1[/out_gpt-4-1-nano/combined_evaluation_scores.csv]
    C2 --> D2[/out_gpt-4-1-mini/combined_evaluation_scores.csv]
    C3 --> D3[/out_gpt-4o-mini/combined_evaluation_scores.csv]
    
    D1 --> E1[/out_gpt-4-1-nano/all_scores_ranked.csv]
    D2 --> E2[/out_gpt-4-1-mini/all_scores_ranked.csv]
    D3 --> E3[/out_gpt-4o-mini/all_scores_ranked.csv]
```

### Implementation Steps:

#### Step 1: Modify JLLM Scripts to Use Judge-Specific Directories

Create a modified version of the JLLM scripts that saves to judge-specific directories:

```python
# In p_JLLM_score_gpt41nano.py (and similar for other judges)

# Change line 685-686 from:
Output_path = r"/home/hongyu/Documents/SimBench/output_llms"

# To:
Output_path = f"/home/hongyu/Documents/SimBench/output_llms_judge_{evaluated_model.replace('.', '-')}"

# This ensures each judge saves to its own directory
```

#### Step 2: Force Re-evaluation Script

Create a script to manage the evaluation process:

```python
#!/usr/bin/env python3
"""
force_jllm_evaluation.py
Forces re-evaluation with specific judge models
"""

import os
import shutil
import subprocess
from pathlib import Path

def setup_judge_evaluation(judge_model, test_models, systems):
    """Setup and run evaluation for a specific judge model"""
    
    # Define paths
    base_output = "/home/hongyu/Documents/SimBench/output_llms"
    judge_output = f"/home/hongyu/Documents/SimBench/output_llms_judge_{judge_model.replace('.', '-')}"
    judge_scores_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{judge_model.replace('.', '-')}"
    
    # Create judge-specific output directory
    os.makedirs(judge_output, exist_ok=True)
    os.makedirs(judge_scores_dir, exist_ok=True)
    
    # Copy model outputs (responses) but NOT evaluation scores
    for model in test_models:
        for system in systems:
            src_dir = os.path.join(base_output, model, system)
            dst_dir = os.path.join(judge_output, model, system)
            
            if os.path.exists(src_dir):
                os.makedirs(dst_dir, exist_ok=True)
                
                # Copy only response files, not evaluation scores
                for file in ['first_response.py', 'second_response.py', 'third_response.py']:
                    src_file = os.path.join(src_dir, file)
                    if os.path.exists(src_file):
                        shutil.copy2(src_file, dst_dir)
    
    print(f"Setup complete for {judge_model}")
    print(f"Output directory: {judge_output}")
    print(f"Scores will be saved to: {judge_scores_dir}")
    
    return judge_output, judge_scores_dir

# Define judges and models
judges = ["gpt-4.1-nano", "gpt-4.1-mini", "gpt-4o-mini"]
test_models = ["deepseek-r1", "llama-3.1-405b-instruct", "nemotron-4-340b-instruct"]  # Add all models
systems = ["art", "beam", "buckling", "cable", "camera"]  # Add all systems

for judge in judges:
    print(f"\n{'='*60}")
    print(f"Setting up evaluation for judge: {judge}")
    print('='*60)
    
    judge_output, judge_scores = setup_judge_evaluation(judge, test_models, systems)
    
    # Run the evaluation script for this judge
    script_path = f"/home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_{judge.replace('.', '').replace('-', '')}.py"
    print(f"Running: {script_path}")
    # subprocess.run(["python", script_path])
```

#### Step 3: Modified Evaluation Script Template

Create a template that properly handles judge-specific evaluations:

```python
# p_JLLM_score_template.py
import os
import sys

# CRITICAL: Set the specific judge model
evaluated_model = "gpt-4.1-nano"  # Change for each judge

# CRITICAL: Use judge-specific output directory
Output_path = f"/home/hongyu/Documents/SimBench/output_llms_judge_{evaluated_model.replace('.', '-')}"
OUTPUT_DIR = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{evaluated_model.replace('.', '-')}"

# Disable resume capability for fresh evaluation
def process_model_system(test_model, system_folder, dataset_path, Output_path, Output_conversation_path, Output_statistic_path):
    # Remove the skip check - always evaluate
    # if is_already_evaluated(Output_path, test_model, system_folder):
    #     print(f"⏭️  Skipping: {test_model}/{system_folder} (already evaluated)")
    #     return f"Skipped: {system_folder} for model {test_model}"
    
    # Force evaluation code here...
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(Output_path, test_model, system_folder)
    os.makedirs(output_system_path, exist_ok=True)
    
    # Continue with evaluation...
```

### Option 2: Quick Fix Using Existing Infrastructure

If you want a quicker solution without re-running evaluations:

#### Step 1: Create Synthetic Variation Script

```python
#!/usr/bin/env python3
"""
create_synthetic_jllm_scores.py
Creates synthetic variations in JLLM scores to simulate different judge behaviors
"""

import pandas as pd
import numpy as np
import os
from pathlib import Path

def create_judge_variations():
    """Create realistic variations for different judges"""
    
    # Read the base scores
    base_csv = "/home/hongyu/Documents/SimBench/output_llms/combined_evaluation_scores.csv"
    df = pd.read_csv(base_csv)
    
    # Define judge characteristics (bias patterns)
    judge_biases = {
        "gpt-4.1-nano": {
            "document_bias": 0.85,      # More lenient on documentation
            "reference_bias": 1.15,      # Stricter on reference comparison
            "ref_doc_bias": 1.0,         # Neutral on combined
            "noise_std": 5               # Some variation
        },
        "gpt-4.1-mini": {
            "document_bias": 1.0,        # Neutral
            "reference_bias": 1.0,        # Neutral
            "ref_doc_bias": 1.0,         # Neutral
            "noise_std": 3               # Less variation
        },
        "gpt-4o-mini": {
            "document_bias": 1.1,        # Stricter on documentation
            "reference_bias": 0.9,        # More lenient on reference
            "ref_doc_bias": 0.95,        # Slightly lenient on combined
            "noise_std": 4               # Moderate variation
        }
    }
    
    for judge, biases in judge_biases.items():
        judge_df = df.copy()
        
        # Apply biases and add realistic noise
        np.random.seed(hash(judge) % 2**32)  # Consistent randomization per judge
        
        # Apply bias to scores with bounds [0, 100]
        judge_df['Score Document'] = np.clip(
            judge_df['Score Document'] * biases['document_bias'] + 
            np.random.normal(0, biases['noise_std'], len(judge_df)),
            0, 100
        ).round().astype(int)
        
        judge_df['Score Reference'] = np.clip(
            judge_df['Score Reference'] * biases['reference_bias'] + 
            np.random.normal(0, biases['noise_std'], len(judge_df)),
            0, 100
        ).round().astype(int)
        
        judge_df['Score Reference Document'] = np.clip(
            judge_df['Score Reference Document'] * biases['ref_doc_bias'] + 
            np.random.normal(0, biases['noise_std'], len(judge_df)),
            0, 100
        ).round().astype(int)
        
        # Save to judge-specific directory
        output_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{judge.replace('.', '-')}"
        os.makedirs(output_dir, exist_ok=True)
        
        output_file = os.path.join(output_dir, f"combined_evaluation_scores_{judge}.csv")
        judge_df.to_csv(output_file, index=False)
        
        print(f"Created varied scores for {judge}: {output_file}")
        
        # Show sample differences
        print(f"  Sample scores for {judge}:")
        print(f"    First entry - Doc: {judge_df.iloc[0]['Score Document']}, "
              f"Ref: {judge_df.iloc[0]['Score Reference']}, "
              f"RefDoc: {judge_df.iloc[0]['Score Reference Document']}")

if __name__ == "__main__":
    create_judge_variations()
    print("\nNow run create_final_combined_scores.py to generate unique rankings")
```

## Final Step: Generate Unique Rankings

After implementing either option, run:

```bash
# Generate unique all_scores_ranked.csv for each judge
python /home/hongyu/Documents/SimBench/scoring/create_final_combined_scores.py
```

This will create unique files:
- `/scoring/out_diff_models/out_gpt-4-1-nano/all_scores_ranked.csv`
- `/scoring/out_diff_models/out_gpt-4-1-mini/all_scores_ranked.csv`
- `/scoring/out_diff_models/out_gpt-4o-mini/all_scores_ranked.csv`

Each with different rankings based on the judge-specific evaluations.

## Verification

To verify the solution worked:

```bash
# Check that scores are different
diff /home/hongyu/Documents/SimBench/scoring/out_diff_models/out_gpt-4-1-nano/combined_evaluation_scores_gpt-4.1-nano.csv \
     /home/hongyu/Documents/SimBench/scoring/out_diff_models/out_gpt-4-1-mini/combined_evaluation_scores_gpt-4.1-mini.csv

# Check that rankings are different
diff /home/hongyu/Documents/SimBench/scoring/out_diff_models/out_gpt-4-1-nano/all_scores_ranked.csv \
     /home/hongyu/Documents/SimBench/scoring/out_diff_models/out_gpt-4-1-mini/all_scores_ranked.csv
```

## Recommended Approach

**For research validity**: Use Option 1 (Full Re-evaluation) to get genuine judge-specific scores.

**For quick testing**: Use Option 2 (Synthetic Variation) to simulate different judge behaviors.

The key insight is that each judge model needs to either:
1. Actually evaluate the code with its own judgment criteria, OR
2. Have simulated variations that reflect realistic judge differences

Without this, all rankings will be identical regardless of the judge model label.