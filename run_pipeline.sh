#!/bin/bash

# SimBench Complete Pipeline Runner
# This script runs the complete evaluation pipeline for all LLM models
# Author: SimBench Team
# Date: 2025

set -e  # Exit on error

# ====================================================================
# CONFIGURATION SECTION - EDIT THESE VALUES
# ====================================================================

# Load API keys from .env file if it exists
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ -f "${SCRIPT_DIR}/.env" ]; then
    echo "Loading API keys from .env file..."
    set -a  # automatically export all variables
    source "${SCRIPT_DIR}/.env"
    set +a
fi

# API Keys - These will be overridden by .env if it exists
export OPENAI_API_KEY="${OPENAI_API_KEY:-}"           # Your OpenAI API key
export ANTHROPIC_API_KEY="${ANTHROPIC_API_KEY:-}"     # Your Anthropic (Claude) API key
export GOOGLE_API_KEY="${GOOGLE_API_KEY:-}"           # Your Google (Gemini) API key
export MISTRAL_API_KEY="${MISTRAL_API_KEY:-}"         # Your Mistral API key
export NVIDIA_API_KEY="${NVIDIA_API_KEY:-}"           # Your NVIDIA API key

# Conda environment name
CONDA_ENV="chrono"

# Base directory (automatically detected)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BASE_DIR="${SCRIPT_DIR}"

# Directories
SCORING_DIR="${BASE_DIR}/scoring"
DATA_DIR="${BASE_DIR}/demo_data"
OUTPUT_DIR="${BASE_DIR}/output_llms"
STATISTIC_DIR="${BASE_DIR}/statistic"
CHECKPOINT_DIR="${BASE_DIR}/.checkpoints"

# Create checkpoint directory if it doesn't exist
mkdir -p "${CHECKPOINT_DIR}"

# ====================================================================
# COLOR CODES FOR OUTPUT
# ====================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ====================================================================
# UTILITY FUNCTIONS
# ====================================================================

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

check_checkpoint() {
    if [ -f "${CHECKPOINT_DIR}/$1" ]; then
        return 0
    else
        return 1
    fi
}

set_checkpoint() {
    touch "${CHECKPOINT_DIR}/$1"
    echo "$(date)" > "${CHECKPOINT_DIR}/$1"
}

clear_checkpoints() {
    rm -rf "${CHECKPOINT_DIR}"
    mkdir -p "${CHECKPOINT_DIR}"
}

check_api_keys() {
    local has_keys=false
    
    if [ -n "$OPENAI_API_KEY" ]; then
        print_success "OpenAI API key configured"
        has_keys=true
    else
        print_warning "OpenAI API key not configured"
    fi
    
    if [ -n "$ANTHROPIC_API_KEY" ]; then
        print_success "Anthropic API key configured"
        has_keys=true
    else
        print_warning "Anthropic API key not configured"
    fi
    
    if [ -n "$GOOGLE_API_KEY" ]; then
        print_success "Google API key configured"
        has_keys=true
    else
        print_warning "Google API key not configured"
    fi
    
    if [ -n "$MISTRAL_API_KEY" ]; then
        print_success "Mistral API key configured"
        has_keys=true
    else
        print_warning "Mistral API key not configured"
    fi
    
    if [ "$has_keys" = false ]; then
        print_error "No API keys configured. Please add at least one API key at the top of this script."
        exit 1
    fi
}

# ====================================================================
# MAIN PIPELINE FUNCTIONS
# ====================================================================

activate_conda() {
    print_header "Activating Conda Environment"
    
    # Source conda - check anaconda3 first since that's where chrono env is
    if [ -f ~/anaconda3/etc/profile.d/conda.sh ]; then
        source ~/anaconda3/etc/profile.d/conda.sh
    elif [ -f ~/miniconda3/etc/profile.d/conda.sh ]; then
        source ~/miniconda3/etc/profile.d/conda.sh
    elif [ -f /opt/conda/etc/profile.d/conda.sh ]; then
        source /opt/conda/etc/profile.d/conda.sh
    else
        print_error "Could not find conda installation"
        exit 1
    fi
    
    # Activate environment
    conda activate ${CONDA_ENV}
    print_success "Activated conda environment: ${CONDA_ENV}"
    
    # Export Python path for the chrono environment
    export PYTHON_CMD="$(which python)"
    print_success "Using Python: ${PYTHON_CMD}"
}

check_dependencies() {
    print_header "Checking Dependencies"
    
    # Check Python version
    python_version=$(${PYTHON_CMD:-python} --version 2>&1 | grep -Po '(?<=Python )\d+\.\d+')
    print_success "Python version: $python_version"
    
    # Check required packages
    ${PYTHON_CMD:-python} -c "import pychrono" 2>/dev/null && print_success "PyChrono installed" || print_error "PyChrono not installed"
    ${PYTHON_CMD:-python} -c "import codebleu" 2>/dev/null && print_success "CodeBLEU installed" || print_error "CodeBLEU not installed"
    ${PYTHON_CMD:-python} -c "import evaluate" 2>/dev/null && print_success "Evaluate installed" || print_error "Evaluate not installed"
    ${PYTHON_CMD:-python} -c "import openai" 2>/dev/null && print_success "OpenAI installed" || print_warning "OpenAI not installed"
    ${PYTHON_CMD:-python} -c "import anthropic" 2>/dev/null && print_success "Anthropic installed" || print_warning "Anthropic not installed"
    
    echo ""
}

clean_ground_truth() {
    print_header "Phase 1: Cleaning Ground Truth Files"
    
    if check_checkpoint "ground_truth_cleaned"; then
        print_warning "Ground truth already cleaned. Skipping..."
        return
    fi
    
    cd "${SCORING_DIR}"
    ${PYTHON_CMD:-python} clean_truth.py
    
    if [ $? -eq 0 ]; then
        set_checkpoint "ground_truth_cleaned"
        print_success "Ground truth files cleaned successfully"
    else
        print_error "Failed to clean ground truth files"
        exit 1
    fi
}

generate_llm_outputs() {
    print_header "Phase 2: Generating LLM Outputs"
    
    cd "${SCORING_DIR}/v01"
    
    # Check which models to run based on available API keys
    if [ -n "$OPENAI_API_KEY" ]; then
        if ! check_checkpoint "openai_generated"; then
            print_header "Generating OpenAI GPT outputs..."
            ${PYTHON_CMD:-python} gpt_generate_simulation.py
            if [ $? -eq 0 ]; then
                set_checkpoint "openai_generated"
                print_success "OpenAI GPT outputs generated"
            else
                print_warning "Failed to generate some OpenAI outputs"
            fi
        else
            print_warning "OpenAI outputs already generated. Skipping..."
        fi
    fi
    
    if [ -n "$ANTHROPIC_API_KEY" ]; then
        if ! check_checkpoint "claude_generated"; then
            print_header "Generating Claude outputs..."
            ${PYTHON_CMD:-python} claude_generate_simulation.py
            if [ $? -eq 0 ]; then
                set_checkpoint "claude_generated"
                print_success "Claude outputs generated"
            else
                print_warning "Failed to generate some Claude outputs"
            fi
        else
            print_warning "Claude outputs already generated. Skipping..."
        fi
    fi
    
    if [ -n "$GOOGLE_API_KEY" ]; then
        if ! check_checkpoint "google_generated"; then
            print_header "Generating Google Gemini outputs..."
            ${PYTHON_CMD:-python} google_generate_simulation.py
            if [ $? -eq 0 ]; then
                set_checkpoint "google_generated"
                print_success "Google Gemini outputs generated"
            else
                print_warning "Failed to generate some Google outputs"
            fi
        else
            print_warning "Google outputs already generated. Skipping..."
        fi
    fi
    
    if [ -n "$MISTRAL_API_KEY" ]; then
        if ! check_checkpoint "mistral_generated"; then
            print_header "Generating Mistral outputs..."
            ${PYTHON_CMD:-python} mistral_generate_simulation.py
            if [ $? -eq 0 ]; then
                set_checkpoint "mistral_generated"
                print_success "Mistral outputs generated"
            else
                print_warning "Failed to generate some Mistral outputs"
            fi
        else
            print_warning "Mistral outputs already generated. Skipping..."
        fi
    fi
}

extract_python_code() {
    print_header "Phase 3: Extracting Python Code from LLM Responses"
    
    if check_checkpoint "code_extracted"; then
        print_warning "Code already extracted. Skipping..."
        return
    fi
    
    cd "${SCORING_DIR}"
    ${PYTHON_CMD:-python} extractPy.py
    
    if [ $? -eq 0 ]; then
        set_checkpoint "code_extracted"
        print_success "Python code extracted successfully"
    else
        print_error "Failed to extract Python code"
        exit 1
    fi
}

evaluate_code() {
    print_header "Phase 4: Evaluating Generated Code"
    
    if check_checkpoint "code_evaluated"; then
        print_warning "Code already evaluated. Skipping..."
        return
    fi
    
    cd "${SCORING_DIR}"
    ${PYTHON_CMD:-python} evaluatePy.py
    
    if [ $? -eq 0 ]; then
        set_checkpoint "code_evaluated"
        print_success "Code evaluation completed"
    else
        print_warning "Some evaluations failed (this is normal)"
        set_checkpoint "code_evaluated"
    fi
}

update_model_discovery() {
    print_header "Phase 4.5: Updating Model Discovery"
    
    cd "${SCORING_DIR}"
    
    # Run auto-discovery to ensure all models are included
    ${PYTHON_CMD:-python} auto_discover_models.py
    
    if [ $? -eq 0 ]; then
        print_success "Model list updated with all discovered models"
    else
        print_warning "Model discovery failed - using existing list"
    fi
}

calculate_scores() {
    print_header "Phase 5: Calculating Similarity Scores"
    
    if check_checkpoint "scores_calculated"; then
        print_warning "Scores already calculated. Skipping..."
        return
    fi
    
    # First update model discovery
    update_model_discovery
    
    cd "${SCORING_DIR}"
    ${PYTHON_CMD:-python} p_sim_score.py
    
    if [ $? -eq 0 ]; then
        set_checkpoint "scores_calculated"
        print_success "Similarity scores calculated"
    else
        print_error "Failed to calculate similarity scores"
        exit 1
    fi
}

generate_rankings() {
    print_header "Phase 6: Generating Rankings"
    
    cd "${SCORING_DIR}"
    
    # Check if required input files exist
    if [ ! -f "${STATISTIC_DIR}/evaluation_results.csv" ]; then
        print_error "evaluation_results.csv not found. Please run scoring first."
        exit 1
    fi
    
    # Run ranking script
    ${PYTHON_CMD:-python} rank_llm.py
    
    if [ $? -eq 0 ]; then
        print_success "Rankings generated successfully"
    else
        print_warning "Ranking failed - some models may be missing"
    fi
}

show_results() {
    print_header "Pipeline Results"
    
    # Check for output files
    if [ -f "${STATISTIC_DIR}/evaluation_results.csv" ]; then
        print_success "Evaluation results saved to: ${STATISTIC_DIR}/evaluation_results.csv"
        
        # Show summary statistics
        echo -e "\n${BLUE}Summary Statistics:${NC}"
        ${PYTHON_CMD:-python} -c "
import pandas as pd
import os

results_file = '${STATISTIC_DIR}/evaluation_results.csv'
if os.path.exists(results_file):
    df = pd.read_csv(results_file)
    print(f'Total evaluations: {len(df)}')
    print(f'Models evaluated: {df[\"model\"].nunique()}')
    print(f'Systems tested: {df[\"system\"].nunique()}')
    
    # Show top models by average CodeBLEU
    if 'codebleu' in df.columns:
        top_models = df.groupby('model')['codebleu'].mean().sort_values(ascending=False).head(5)
        print('\nTop 5 Models by Average CodeBLEU:')
        for model, score in top_models.items():
            print(f'  {model}: {score:.4f}')
"
    fi
    
    if [ -f "${SCORING_DIR}/out/consensus_llm_rankings.csv" ]; then
        print_success "Consensus rankings saved to: ${SCORING_DIR}/out/consensus_llm_rankings.csv"
        
        echo -e "\n${BLUE}Top 10 Models (Consensus Ranking):${NC}"
        head -n 11 "${SCORING_DIR}/out/consensus_llm_rankings.csv" | column -t -s ','
    fi
    
    if [ -f "${SCORING_DIR}/out/consensus_llm_rankings_top10.png" ]; then
        print_success "Visualization saved to: ${SCORING_DIR}/out/consensus_llm_rankings_top10.png"
    fi
}

verify_model_coverage() {
    print_header "Verifying Model Coverage"
    
    # List of all models from p_sim_score.py
    expected_models=(
        "gemma-2-2b-it" "gemma-2-9b-it" "gemma-2-27b-it"
        "llama-3.1-405b-instruct" "llama-3.1-70b-instruct" "llama-3.1-8b-instruct"
        "gpt-4o" "gpt-4o-mini" "claude-3-5-sonnet"
        "Gemini-1.5-pro" "gpt-4.1" "gpt-4.1-mini"
    )
    
    echo "Checking for model outputs..."
    for model in "${expected_models[@]}"; do
        if [ -d "${OUTPUT_DIR}/${model}" ]; then
            count=$(find "${OUTPUT_DIR}/${model}" -name "*.py" 2>/dev/null | wc -l)
            if [ $count -gt 0 ]; then
                print_success "$model: $count Python files found"
            else
                print_warning "$model: Directory exists but no Python files"
            fi
        else
            print_warning "$model: No output directory found"
        fi
    done
}

# ====================================================================
# MAIN EXECUTION
# ====================================================================

main() {
    clear
    print_header "SimBench Complete Pipeline Runner"
    
    # Parse command line arguments
    case "${1:-}" in
        --clean)
            print_header "Clearing All Checkpoints"
            clear_checkpoints
            print_success "All checkpoints cleared. Pipeline will run from beginning."
            ;;
        --skip-generation)
            SKIP_GENERATION=true
            print_warning "Skipping LLM generation phase"
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --clean           Clear all checkpoints and run from beginning"
            echo "  --skip-generation Skip LLM generation phase (use existing outputs)"
            echo "  --help           Show this help message"
            echo ""
            echo "Edit this script to add your API keys before running."
            exit 0
            ;;
    esac
    
    # Check API keys
    if [ "${SKIP_GENERATION:-false}" != "true" ]; then
        check_api_keys
    fi
    
    # Activate conda environment
    activate_conda
    
    # Check dependencies
    check_dependencies
    
    # Validate pipeline before running
    print_header "Validating Pipeline Configuration"
    cd "${SCORING_DIR}"
    ${PYTHON_CMD:-python} validate_pipeline.py
    if [ $? -ne 0 ]; then
        print_error "Pipeline validation failed. Please fix issues before running."
        print_warning "Run: python ${SCORING_DIR}/validate_pipeline.py for details"
        exit 1
    fi
    print_success "Pipeline validation passed"
    
    # Run pipeline phases
    clean_ground_truth
    
    if [ "${SKIP_GENERATION:-false}" != "true" ]; then
        generate_llm_outputs
    else
        print_warning "Skipping LLM generation phase as requested"
    fi
    
    extract_python_code
    evaluate_code
    calculate_scores
    generate_rankings
    
    # Verify coverage
    verify_model_coverage
    
    # Show results
    show_results
    
    print_header "Pipeline Complete!"
    print_success "All phases completed successfully"
    echo -e "\n${GREEN}Checkpoints are saved in: ${CHECKPOINT_DIR}${NC}"
    echo -e "${GREEN}To re-run from beginning, use: $0 --clean${NC}"
    echo -e "${GREEN}To skip LLM generation, use: $0 --skip-generation${NC}"
}

# Run main function
main "$@"