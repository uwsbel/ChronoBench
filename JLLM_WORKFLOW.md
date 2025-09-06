# SimBench JLLM Evaluation Workflow Guide

This document provides a complete guide for running the Clean → Simulate → Extract → Evaluate → Score Rank process for the 3 Judge LLMs (JLLMs) in the SimBench repository.

## Overview

SimBench evaluates simulation code generation capabilities using three OpenAI Judge LLMs:
1. **gpt-4o-mini** - Standard evaluation model
2. **gpt-4.1-mini** - Enhanced mini model 
3. **gpt-4.1-nano** - Lightweight nano model

## Prerequisites

### 1. Environment Setup
```bash
# Create and activate conda environment
conda create -n chrono python=3.9
conda activate chrono

# Install dependencies
pip install -r requirements.txt
```

### 2. API Keys Configuration
Create a `.env` file in the repository root with your API keys:
```bash
# OpenAI API Keys (3 separate keys for parallel execution)
OPENAI_API_KEY_1="your-api-key-1"
OPENAI_API_KEY_2="your-api-key-2"  
OPENAI_API_KEY_3="your-api-key-3"

# Other API Keys (if needed)
NVIDIA_API_KEY="your-nvidia-key"
ANTHROPIC_API_KEY="your-anthropic-key"
GOOGLE_API_KEY="your-google-key"
MISTRAL_API_KEY="your-mistral-key"
```

## Complete Workflow Steps

### Step 1: Clean Ground Truth Data
Remove comments from ground truth demos to prepare for similarity evaluation:

```bash
cd scoring
python clean_truth.py
```

This processes files in `demo_data/` folder and creates cleaned versions.

### Step 2: Simulate - Generate LLM Outputs
Run simulation tasks to generate outputs from S-LLMs (Simulation LLMs):

```bash
# Run the main pipeline for S-LLM generation
./run_pipeline.sh
```

This will:
- Generate simulation code for all 34 systems
- Create outputs in `output_llms/` directory
- Each system has 3 complexity levels (first, second, third)

### Step 3: Extract Python Code
Extract and clean Python code from S-LLM outputs:

```bash
cd scoring
python extractPy.py
```

This:
- Removes comments and non-code content
- Extracts pure Python code from responses
- Saves cleaned files as `*_cleaned_response.py`

### Step 4: Evaluate Code Execution
Test if generated code compiles and runs:

```bash
python evaluatePy.py
```

This:
- Attempts to compile each Python script
- Runs the code with timeout protection
- Records execution success/failure
- Generates execution logs in `execution.log`

### Step 5: Score with 3 JLLMs (Parallel)
Run all three Judge LLMs in parallel for scoring:

```bash
cd scoring
./run_multiple_openaijllm.sh
```

This script:
- Launches 3 JLLM evaluations in parallel
- Each JLLM uses a separate API key to avoid rate limits
- Monitors progress in real-time
- Creates outputs in `out_diff_models/out_[model-name]/`

#### Individual JLLM Scripts (if needed):
```bash
# Run individually from scoring/v01/ directory
cd scoring/v01

# For gpt-4o-mini
python p_JLLM_score_gpt4omini.py

# For gpt-4.1-mini  
python p_JLLM_score_gpt41mini.py

# For gpt-4.1-nano
python p_JLLM_score_gpt41nano.py
```

### Step 6: Generate Rankings
After all JLLMs complete evaluation, generate final rankings:

```bash
cd scoring

# Generate rankings for each JLLM
python generate_rankings_for_judges.py

# Generate combined JLLM rankings
python generate_jllm_all_scores_ranked.py

# Create final combined scores
python create_final_combined_scores.py
```

## Output Structure

```
SimBench/
├── output_llms/                    # S-LLM generated code
│   ├── [model_name]/
│   │   └── [system_name]/
│   │       ├── first_response.py
│   │       ├── second_response.py
│   │       └── third_response.py
│
├── scoring/
│   ├── out_diff_models/           # JLLM evaluation results
│   │   ├── out_gpt-4o-mini/
│   │   │   ├── jllm_score_log.txt
│   │   │   └── evaluation_scores.csv
│   │   ├── out_gpt-4-1-mini/
│   │   └── out_gpt-4-1-nano/
│   │
│   └── out/                       # Final rankings
│       ├── all_scores_ranked.csv
│       └── jllm_combined_rankings.csv
```

## Monitoring Progress

### Check JLLM Logs in Real-time
```bash
# Monitor specific JLLM progress
tail -f scoring/out_diff_models/out_gpt-4o-mini/jllm_score_log.txt
tail -f scoring/out_diff_models/out_gpt-4-1-mini/jllm_score_log.txt
tail -f scoring/out_diff_models/out_gpt-4-1-nano/jllm_score_log.txt
```

### Check Process Status
```bash
# See running Python processes
ps aux | grep python | grep JLLM
```

## Troubleshooting

### Rate Limit Issues
- Use separate API keys for each JLLM (OPENAI_API_KEY_1, 2, 3)
- The scripts automatically handle rate limits with exponential backoff
- Check logs for "Rate limit" messages

### Failed Evaluations
Check for failed evaluations:
```bash
cd scoring
python analyze_failed_scores.py
```

Fix specific errors:
```bash
python fix_error_files.py
python fix_math_expression_scores.py
```

### Missing Scores
Compute missing metrics:
```bash
python compute_missing_pe_metrics.py
```

## System Categories

The evaluation covers 34 systems across 5 categories:

- **MBS (Multibody Systems)**: gear, mass_spring_damper, particles, pendulum, slider_crank
- **FEA (Finite Element Analysis)**: beam, buckling, cable, rotor, tablecloth  
- **SEN (Sensors)**: camera, gps_imu, lidar, veh_app
- **RBT (Robotics)**: curiosity, handler, viper, turtlebot, vehros, sensros
- **VEH (Vehicles)**: art, citybus, feda, gator, hmmwv, kraz, m113, man, rigid_highway, rigid_multipatches, scm, scm_hill, sedan, uazbus

## Complete One-Command Execution

For a fully automated run (requires proper .env setup):
```bash
# From repository root
./run_complete_evaluation.sh
```

Create this script if needed:
```bash
#!/bin/bash
set -e

echo "Starting complete SimBench evaluation..."

# Step 1: Clean
cd scoring
python clean_truth.py

# Step 2: Extract
python extractPy.py

# Step 3: Evaluate
python evaluatePy.py

# Step 4: Score with JLLMs
./run_multiple_openaijllm.sh

# Wait for completion
echo "Waiting for JLLM evaluations to complete..."
while pgrep -f "p_JLLM_score" > /dev/null; do
    sleep 60
done

# Step 5: Generate rankings
python generate_rankings_for_judges.py
python generate_jllm_all_scores_ranked.py
python create_final_combined_scores.py

echo "Complete evaluation finished!"
echo "Results available in scoring/out/"
```

## Important Notes

1. **API Keys**: Each JLLM should use a separate API key to avoid rate limits
2. **Parallel Execution**: The 3 JLLMs run in parallel for efficiency
3. **Checkpoint System**: The pipeline supports checkpoints for resuming interrupted runs
4. **Logging**: All operations are logged for debugging and monitoring
5. **Timeout Protection**: Code execution has timeout limits to prevent hanging

## Contact

For issues or questions about the evaluation pipeline, check the logs in:
- `scoring/execution.log` - Code execution results
- `scoring/extraction.log` - Code extraction process
- `scoring/out_diff_models/*/jllm_score_log.txt` - JLLM evaluation logs