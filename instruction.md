# SimBench Repository - Complete Setup and Execution Guide

## Quick Navigation
1. **[Prerequisites](#prerequisites---must-complete-before-running-pipeline)** - MUST complete first!
2. **[Automated Pipeline](#automated-pipeline-execution-recommended)** - Easiest way to run everything
3. **[Manual Pipeline](#manual-pipeline---step-by-step-alternative-method)** - For debugging or custom runs
4. **[Quick Start](#quick-start-guide)** - TL;DR version
5. **[Troubleshooting](#troubleshooting)** - Common issues and solutions

## ⚠️ IMPORTANT: Setup Order
1. **FIRST**: Complete ALL prerequisites (environment, dependencies, API keys)
2. **THEN**: Run the automated pipeline script (`./run_pipeline.sh`)
3. **OR**: Follow the manual pipeline for step-by-step control

## Overview
SimBench is a benchmark designed to evaluate large language models (LLMs) in generating Digital Twins (DTs) for simulators. It uses a Judge-LLM (J-LLM) to assess the quality of simulations created by Student-LLMs (S-LLMs), specifically for the PyChrono multi-physics simulator.

## Repository Structure
```
SimBench/
├── api/                    # API documentation for metrics evaluation
│   └── api.txt            # Consolidated API reference (~4000 tokens)
├── demo_data/             # Ground truth simulation demos (34 systems)
│   ├── art/              # Articulated vehicle simulations
│   ├── beam/             # Beam FEA simulations
│   ├── vehicle demos/    # Various vehicle dynamics (citybus, hmmwv, sedan, etc.)
│   ├── robotics/         # Robot simulations (turtlebot, curiosity, viper)
│   └── sensors/          # Sensor integration demos (camera, lidar, gps_imu)
├── output_conversion/     # S-LLM conversations in Stanford Alpaca JSON format
├── output_llms/          # Raw LLM outputs and extracted Python code
├── scoring/              # Evaluation scripts and metrics
│   ├── clean_truth.py   # Remove comments from ground truth
│   ├── extractPy.py     # Extract Python from LLM outputs
│   ├── evaluatePy.py    # Compile and run generated scripts
│   ├── p_sim_score.py   # Calculate similarity metrics (ROUGE, CodeBLEU)
│   ├── rank_llm.py      # Generate rankings and consensus scores
│   └── v01/             # LLM generation scripts for different providers
├── statistic/           # Statistical analysis results
└── visualization/       # Visualization tools and outputs
```

## Prerequisites - MUST COMPLETE BEFORE RUNNING PIPELINE

⚠️ **IMPORTANT**: Complete ALL steps in this section before attempting to run the automated pipeline script!

### Pre-flight Checklist
Before running `run_pipeline.sh`, ensure you have:
- [ ] Installed Conda/Miniconda
- [ ] Created and activated the chrono environment
- [ ] Installed PyChrono successfully
- [ ] Installed all required Python packages with correct versions
- [ ] Obtained API keys for at least one LLM provider
- [ ] Edited run_pipeline.sh to add your API keys

### Required Software
1. **Python 3.8+** (Python 3.9 recommended for best compatibility)
2. **Conda** - For environment management
3. **PyChrono** - The Chrono physics simulation library

### Step-by-Step Environment Setup

#### Step 1: Create or Use Existing Chrono Environment
```bash
# If you have an existing chrono environment
conda activate chrono

# Or create a new one
conda create -n chrono python=3.9
conda activate chrono
```

#### Step 2: Install PyChrono
```bash
# Install PyChrono (recommended via conda)
conda install -c projectchrono pychrono

# Verify installation
python -c "import pychrono; print('PyChrono installed successfully')"
```

#### Step 3: Install ALL Required Python Packages
```bash
# Core dependencies with specific versions that work together
pip install --upgrade pip

# Install evaluation metrics packages (EXACT versions required!)
pip install tree-sitter==0.20.4
pip install codebleu==0.4.0
pip install datasets==4.0.0
pip install evaluate
pip install rouge-score

# Install LLM API clients (install all even if not using)
pip install openai
pip install anthropic
pip install google-generativeai
pip install mistralai  # For Mistral API

# Other required packages
pip install numpy pandas matplotlib tqdm

# Verify critical packages
python -c "import codebleu; print('CodeBLEU installed')"
python -c "import evaluate; print('Evaluate installed')"
```

### Step 4: Prepare API Keys

#### Option A: Obtain API Keys from Providers
1. **OpenAI**: Get key from https://platform.openai.com/api-keys
2. **Anthropic (Claude)**: Get key from https://console.anthropic.com/
3. **Google (Gemini)**: Get key from https://makersuite.google.com/app/apikey
4. **Mistral**: Get key from https://console.mistral.ai/

#### Option B: Configure API Keys in Script
Edit `run_pipeline.sh` and add your keys:
```bash
# Open the script
nano run_pipeline.sh

# Find these lines near the top and add your keys:
export OPENAI_API_KEY="sk-..."           # Your OpenAI API key
export ANTHROPIC_API_KEY="sk-ant-..."    # Your Anthropic API key
export GOOGLE_API_KEY="AIza..."          # Your Google API key
export MISTRAL_API_KEY="..."             # Your Mistral API key
```

**Note**: You need at least ONE API key to generate new outputs. If you're only evaluating existing outputs, you can skip API key configuration and use `--skip-generation` flag.

### Step 5: Verify Environment Setup
```bash
# Run this verification script
python -c "
import sys
print('Python version:', sys.version)
try:
    import pychrono
    print('✓ PyChrono installed')
except: 
    print('✗ PyChrono NOT installed')
try:
    import codebleu
    print('✓ CodeBLEU installed')
except:
    print('✗ CodeBLEU NOT installed')
try:
    import evaluate
    print('✓ Evaluate installed')
except:
    print('✗ Evaluate NOT installed')
try:
    import openai
    print('✓ OpenAI client installed')
except:
    print('✗ OpenAI client NOT installed')
"
```

### Important Version Notes
- **tree-sitter**: MUST be version 0.20.4 for CodeBLEU compatibility
- **datasets**: Version 4.0.0 or higher to avoid PyArrow issues
- **codebleu**: Version 0.4.0 works with tree-sitter 0.20.4
- **Python**: 3.8+ required, 3.9 recommended

## Automated Pipeline Execution (Recommended)

After completing ALL prerequisites above, you can run the entire pipeline automatically using the provided shell script.

### Using the Automated Pipeline Script

The `run_pipeline.sh` script automates the entire evaluation pipeline with checkpoint tracking and error recovery.

#### Basic Usage
```bash
# Make script executable (first time only)
chmod +x run_pipeline.sh

# Run complete pipeline
./run_pipeline.sh

# Run with specific options
./run_pipeline.sh --clean            # Clear checkpoints and start fresh
./run_pipeline.sh --skip-generation  # Skip LLM generation, use existing outputs
./run_pipeline.sh --help            # Show help message
```

#### What the Script Does

1. **Validates Environment**: Checks conda, dependencies, and API keys
2. **Runs All Pipeline Phases**:
   - Phase 1: Cleans ground truth files
   - Phase 2: Generates LLM outputs (if API keys provided)
   - Phase 3: Extracts Python code from responses
   - Phase 4: Evaluates generated code
   - Phase 5: Calculates similarity scores
   - Phase 6: Generates rankings and visualizations
3. **Tracks Progress**: Uses checkpoints to resume if interrupted
4. **Provides Results Summary**: Shows top models and statistics

#### Checkpoint System

The script creates a `.checkpoints/` directory to track completed phases:
- If interrupted, simply run again to resume
- Use `--clean` to start over from the beginning
- Checkpoints prevent unnecessary re-processing

#### Output Files

After successful completion, you'll find:
- `statistic/evaluation_results.csv` - All evaluation metrics
- `scoring/out/consensus_llm_rankings.csv` - Final model rankings
- `scoring/out/consensus_llm_rankings_top10.png` - Top 10 visualization
- `scoring/out/llm_all_metrics_with_rank.csv` - Complete metrics table

#### Troubleshooting Script Issues

If the script fails:
1. Check error messages - they're color-coded (red = error, yellow = warning)
2. Verify all prerequisites are completed
3. Check API keys are correctly formatted (no spaces or newlines)
4. Use `--skip-generation` if you only want to evaluate existing outputs
5. Check `.checkpoints/` to see which phases completed

### When to Use Manual Pipeline

Use the manual pipeline (described below) when you need to:
- Debug specific phases
- Run only certain steps
- Customize parameters
- Understand the process in detail

## Manual Pipeline - Step by Step (Alternative Method)

### Phase 1: Data Preparation

#### Step 1.1: Clean Ground Truth Files
Remove comments from ground truth files to create clean reference code.
```bash
cd /home/hongyu/Documents/SimBench/scoring
python clean_truth.py
```

**What it does:**
- Iterates through each system folder in `demo_data/`
- Removes comments from truth1.py, truth2.py, truth3.py
- Creates cleaned_truth1.py, cleaned_truth2.py, cleaned_truth3.py
- Saves extraction messages to `extraction_message.txt`

**Output:** Cleaned truth files in each demo_data subfolder

### Phase 2: LLM Generation (Optional)

#### Step 2.1: Generate Simulations with LLMs
Choose your LLM provider and generate simulations:

```bash
cd /home/hongyu/Documents/SimBench/scoring/v01

# For OpenAI GPT models
python gpt_generate_simulation.py

# For Claude models
python claude_generate_simulation.py

# For Google models
python google_generate_simulation.py

# For open-source models via vLLM
python vllm_generation.py
```

**What it does:**
- Reads input prompts from demo_data (input1.txt, input2.txt, input3.txt)
- Sends prompts to LLM for three turns of interaction
- Turn 1: Generate initial simulation from text prompt
- Turn 2: Fix errors and modify based on new requirements
- Turn 3: Further refinements and corrections
- Saves raw responses to `output_llms/<model_name>/<system>/`

**Output:**
- first_response.txt, second_response.txt, third_response.txt
- Conversation JSON files in output_conversion/

### Phase 3: Code Extraction

#### Step 3.1: Extract Python Code from LLM Responses
```bash
cd /home/hongyu/Documents/SimBench/scoring
python extractPy.py
```

**What it does:**
- Reads LLM text responses from output_llms/
- Extracts Python code blocks (between ```python and ```)
- Handles multiple code blocks and missing delimiters
- Creates first_response.py, second_response.py, third_response.py
- Also creates cleaned versions without comments

**Output:** 
- Python files extracted from LLM responses
- Cleaned Python files without comments
- extraction_message.txt with status logs

### Phase 4: Evaluation

#### Step 4.1: Test Generated Code
```bash
cd /home/hongyu/Documents/SimBench/scoring
python evaluatePy.py
```

**What it does:**
- Attempts to compile and run each generated Python script
- Tests against PyChrono imports and basic functionality
- Records success/failure for each simulation
- Logs detailed error messages for debugging

**Output:**
- execution.log files with detailed results
- Console output showing pass/fail status

#### Step 4.2: Calculate Similarity Metrics
```bash
cd /home/hongyu/Documents/SimBench/scoring
python p_sim_score.py
```

**What it does:**
- Compares generated code against ground truth using:
  - **ROUGE scores**: Text-level similarity metrics
    - ROUGE-1: Unigram overlap
    - ROUGE-2: Bigram overlap
    - ROUGE-L: Longest common subsequence
  - **CodeBLEU scores**: Code-specific similarity with 4 components:
    - N-gram match: Token-level similarity
    - Weighted n-gram: Emphasis on important keywords
    - Syntax match: AST (Abstract Syntax Tree) structure similarity
    - Dataflow match: Variable usage and flow patterns

**Output:**
- evaluation_results.csv in statistic/ folder
- Individual score files for each model/system

### Phase 5: Analysis and Ranking

#### Step 5.1: Generate Rankings
```bash
cd /home/hongyu/Documents/SimBench/scoring
python rank_llm.py
```

**What it does:**
- Aggregates all evaluation metrics
- Creates rankings based on:
  - Compilation success rate
  - Code similarity scores (ROUGE and CodeBLEU)
  - Overall performance across all systems
- Generates consensus rankings using z-score normalization
- Creates comprehensive metrics table

**Output:**
- `statistic/evaluation_results.csv` - Raw evaluation data
- `out/llm_all_metrics_with_rank.csv` - All metrics with rankings
- `out/consensus_llm_rankings.csv` - Final consensus rankings
- `out/consensus_llm_rankings_top10.png` - Visualization

### Phase 6: Visualization (Optional)

#### Step 6.1: Visualize Results
```bash
cd /home/hongyu/Documents/SimBench/visualization
python plot_results.py  # If available
```

## Quick Start Guide

### Fastest Way - Using Automated Script

**Prerequisites**: Complete ALL setup steps in the Prerequisites section first!

```bash
# 1. Add your API keys to the script
nano run_pipeline.sh  # Add keys at the top of the file

# 2. Run the complete pipeline
./run_pipeline.sh

# Or, if you only want to evaluate existing outputs:
./run_pipeline.sh --skip-generation
```

### Manual Quick Start - Minimal Pipeline

If you want to manually evaluate existing LLM outputs without generation:

```bash
# Navigate to scoring directory
cd /home/hongyu/Documents/SimBench/scoring

# 1. Clean ground truth (only needed once)
python clean_truth.py

# 2. Extract Python from existing LLM outputs
python extractPy.py

# 3. Evaluate the extracted code
python evaluatePy.py

# 4. Calculate similarity scores
python p_sim_score.py

# 5. Generate rankings
python rank_llm.py

# 6. View results
cat statistic/evaluation_results.csv
cat out/consensus_llm_rankings.csv
```

## Understanding the Metrics

### ROUGE Scores
- **ROUGE-1**: Measures unigram (single word) overlap
- **ROUGE-2**: Measures bigram (two-word sequence) overlap
- **ROUGE-L**: Measures longest common subsequence
- **ROUGE-Lsum**: Summary-level ROUGE-L

### CodeBLEU Components
1. **N-gram Match Score**: Basic token overlap (0.25 weight)
2. **Weighted N-gram Score**: Emphasizes keywords (0.25 weight)
3. **Syntax Match Score**: AST structure similarity (0.25 weight)
4. **Dataflow Match Score**: Variable usage patterns (0.25 weight)

**Important Note on Dataflow**: The dataflow score may be 0 for simple simulation setup code that primarily consists of initialization and configuration calls without complex variable flows. This is normal and not an error.

## Troubleshooting

### Common Issues and Solutions

#### 1. ImportError with PyArrow
**Error**: `AttributeError: module 'pyarrow' has no attribute 'PyExtensionType'`
**Solution**:
```bash
pip install --upgrade datasets==4.0.0
```

#### 2. CodeBLEU Tree-sitter Error
**Error**: `TypeError: an integer is required`
**Solution**:
```bash
pip install tree-sitter==0.20.4 codebleu==0.4.0
```

#### 3. NotADirectoryError
**Error**: Script tries to process non-directory files in demo_data/
**Solution**: Ensure scripts check `os.path.isdir()` before processing

#### 4. API Key Errors
**Error**: Missing or invalid API key
**Solution**:
- Check environment variables are set correctly
- Ensure API key doesn't have newlines or spaces
- Verify API key is valid and has credits

#### 5. PyChrono Import Errors
**Error**: `ModuleNotFoundError: No module named 'pychrono'`
**Solution**:
```bash
conda activate chrono
conda install -c projectchrono pychrono
```

#### 6. Dataflow Score is Zero
**Observation**: CodeBLEU dataflow_match_score is 0.0
**Explanation**: This is normal for simulation setup code that mainly consists of:
- Object initialization
- Parameter configuration
- Simple method calls
- No complex variable transformations

The dataflow metric is designed for code with variable transformations and flows, which simulation setup code often lacks.

#### 7. PyTorch/TensorFlow Warnings
**Warning**: `transformers` package warnings about missing PyTorch/TensorFlow
**Explanation**: These are harmless - the evaluate library imports transformers but doesn't actually need PyTorch/TensorFlow for ROUGE/CodeBLEU calculations.
**To suppress**:
```python
import os
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"
import logging
logging.getLogger("transformers").setLevel(logging.ERROR)
```

## Data Flow Diagram

```
demo_data/ (ground truth)
    ↓
[clean_truth.py] → cleaned_truth*.py files
    ↓
[LLM generation] → output_llms/*/response.txt
    ↓
[extractPy.py] → output_llms/*/response.py files
    ↓
[evaluatePy.py] → execution logs & pass/fail status
    ↓
[p_sim_score.py] → similarity scores & metrics
    ↓
[rank_llm.py] → rankings & consensus scores
    ↓
[visualization] → plots and charts
```

## System Categories

The 34 simulation systems are categorized as:

- **MBS (Multi-Body Systems)**: pendulum, slider_crank, gear, mass_spring_damper, particles
- **FEA (Finite Element Analysis)**: beam, buckling, rotor, tablecloth, cable
- **SEN (Sensors)**: gps_imu, lidar, veh_app, camera
- **RBT (Robotics)**: turtlebot, viper, curiosity, vehros, sensros, handler
- **VEH (Vehicles)**: citybus, feda, gator, hmmwv, kraz, art, rigid_highway, rigid_multipatches, scm, scm_hill, uazbus, m113, sedan, man

## Best Practices

1. **Always activate the chrono environment** before running scripts
2. **Run clean_truth.py first** to prepare ground truth data
3. **Check API keys** are properly set if generating new LLM outputs
4. **Use absolute paths** in scripts to avoid path issues
5. **Monitor execution logs** for detailed error messages
6. **Understand warnings vs errors** - dataflow warnings are often expected

## Citation
If using SimBench, please cite:
```bibtex
@article{jingquanSimbench2024,
  title={{SimBench: A Rule-Based Multi-Turn Interaction Benchmark for Evaluating an LLM's Ability to Generate Digital Twins}},
  author={Jingquan Wang and Harry Zhang and Huzaifa Mustafa Unjhawala and Peter Negrut and Shu Wang and Khailanii Slaton and Radu Serban and Jin-Long Wu and Dan Negrut},
  journal={arXiv preprint arXiv:2408.11987},
  year={2024}
}
```