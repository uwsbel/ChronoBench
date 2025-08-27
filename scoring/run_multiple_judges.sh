#!/bin/bash

# Script to run evaluations with different judge LLMs
# This script modifies the judge model in p_JLLM_score.py and runs the complete evaluation pipeline
# Results are stored in out_diff_models/out_<judge_model>/

# Set working directory
SCORING_DIR="/home/hongyu/Documents/SimBench/scoring"
OUTPUT_BASE_DIR="${SCORING_DIR}/out_diff_models"
JLLM_SCRIPT="${SCORING_DIR}/v01/p_JLLM_score.py"
BACKUP_FILE="${JLLM_SCRIPT}.backup"
OUTPUT_LLMS_DIR="/home/hongyu/Documents/SimBench/output_llms"
STATISTIC_DIR="/home/hongyu/Documents/SimBench/statistic"

# Load API keys from .env file if it exists
ENV_FILE="${SCORING_DIR}/../.env"
if [ -f "$ENV_FILE" ]; then
    set -a  # automatically export all variables
    source "$ENV_FILE"
    set +a
    echo "✓ API keys loaded from .env file"
else
    echo "⚠ Warning: .env file not found at $ENV_FILE"
fi

# Export API Keys (will use .env values if loaded, empty otherwise)
export OPENAI_API_KEY="${OPENAI_API_KEY:-}"
export ANTHROPIC_API_KEY="${ANTHROPIC_API_KEY:-}"
export GOOGLE_API_KEY="${GOOGLE_API_KEY:-}"
export MISTRAL_API_KEY="${MISTRAL_API_KEY:-}"
export NVIDIA_API_KEY="${NVIDIA_API_KEY:-}"
export GEMINI_API_KEY="${GOOGLE_API_KEY}"  # Google Gemini uses the same key

# Define judge models to test
# Including all available models from p_JLLM_score.py
JUDGE_MODELS=(
    # OpenAI Models
    "gpt-4o-mini"
    "gpt-4.1-mini"
    "gpt-4.1-nano"

    # Anthropic Models
    "claude-3-5-sonnet"
    "claude-3-7-sonnet-20250219"
    "claude-4-sonnet-20250514"
    
    # Google Models
    "Gemini-1.5-pro"
    "Gemini-2.5-pro"
    "gemma-2-2b-it"
    "gemma-2-9b-it"
    "gemma-2-27b-it"
    "gemma-3-1b-it"
    
    # Meta/Llama Models
    "llama-3.1-405b-instruct"
    "llama-3.1-70b-instruct"
    "llama-3.1-8b-instruct"
    "llama-3.3-70b-instruct"
    "llama4_maverick"
    "llama4_scout"
    "llama3.1-8b-f2"
    "llama3.3-70b-sft1"
    "llama3.1-8b-lora1"
    "llama4-109b-lora1"
    "llama3.3-70b-lora1"
    
    # DeepSeek Models
    "deepseek-r1"
    "deepseek-r1-8b"
    "deepseek-r1-32b"
    
    # Mistral Models
    "mistral-nemo-12b-instruct"
    "mixtral-8x22b-instruct-v0.1"
    "mixtral-8x7b-instruct-v0.1"
    "mistral-large-latest"
    "codestral-22b-instruct-v0.1"
    "mamba-codestral-7b-v0.1"
    
    # NVIDIA Models
    "nemotron-4-340b-instruct"
    
    # Microsoft Phi Models
    "phi-3-mini-128k-instruct"
    "phi-3-medium-128k-instruct"
    
    # Qwen Model
    "qwen3-235b-a22b"
)

# Map models to their API providers
declare -A MODEL_PROVIDER
# OpenAI models
for model in "gpt-4o" "gpt-4o-mini" "gpt-4.1" "gpt-4.1-mini" "gpt-4.1-nano" "o3" "o4-mini"; do
    MODEL_PROVIDER[$model]="openai"
done
# Anthropic models
for model in "claude-3-5-sonnet" "claude-3-7-sonnet-20250219" "claude-4-sonnet-20250514"; do
    MODEL_PROVIDER[$model]="anthropic"
done
# Google models
for model in "Gemini-1.5-pro" "Gemini-2.5-pro" "gemma-2-2b-it" "gemma-2-9b-it" "gemma-2-27b-it" "gemma-3-1b-it"; do
    MODEL_PROVIDER[$model]="google"
done
# Mistral models
for model in "mistral-nemo-12b-instruct" "mixtral-8x22b-instruct-v0.1" "mixtral-8x7b-instruct-v0.1" "mistral-large-latest" "codestral-22b-instruct-v0.1" "mamba-codestral-7b-v0.1"; do
    MODEL_PROVIDER[$model]="mistral"
done
# NVIDIA/Meta models (often use NVIDIA endpoints)
for model in "llama-3.1-405b-instruct" "llama-3.1-70b-instruct" "llama-3.1-8b-instruct" "llama-3.3-70b-instruct" "llama4_maverick" "llama4_scout" "llama3.1-8b-f2" "llama3.3-70b-sft1" "llama3.1-8b-lora1" "llama4-109b-lora1" "llama3.3-70b-lora1" "nemotron-4-340b-instruct" "phi-3-mini-128k-instruct" "phi-3-medium-128k-instruct"; do
    MODEL_PROVIDER[$model]="nvidia"
done
# DeepSeek models
for model in "deepseek-r1" "deepseek-r1-8b" "deepseek-r1-32b"; do
    MODEL_PROVIDER[$model]="deepseek"
done
# Qwen model
MODEL_PROVIDER["qwen3-235b-a22b"]="qwen"

# Track failed API providers
declare -A FAILED_PROVIDERS
declare -A PROVIDER_FAILURES

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Spinner function for API wait with custom message
spinner() {
    local pid=$1
    local msg="${2:-Waiting for API response...}"
    local delay=0.1
    local spinstr='⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'
    echo -n " "
    while [ "$(ps a | awk '{print $1}' | grep $pid)" ]; do
        local temp=${spinstr#?}
        printf " [%c] %s" "$spinstr" "$msg"
        local spinstr=$temp${spinstr%"$temp"}
        sleep $delay
        printf "\r"
    done
    printf "    \r"
}

# Function to check if required files exist
check_prerequisites() {
    local missing_files=0
    
    print_message "$YELLOW" "Checking prerequisites..."
    
    # Check if LLM outputs exist
    if [ ! -d "$OUTPUT_LLMS_DIR" ]; then
        print_message "$RED" "Error: Output directory $OUTPUT_LLMS_DIR not found"
        missing_files=1
    fi
    
    # Check if demo data exists
    if [ ! -d "/home/hongyu/Documents/SimBench/demo_data" ]; then
        print_message "$RED" "Error: Demo data directory not found"
        missing_files=1
    fi
    
    # Check if API documentation exists
    if [ ! -f "/home/hongyu/Documents/SimBench/api/api.txt" ]; then
        print_message "$RED" "Error: API documentation not found"
        missing_files=1
    fi
    
    # Check if p_sim_score.py has been run (evaluation_results.csv should exist)
    if [ ! -f "$STATISTIC_DIR/evaluation_results.csv" ]; then
        print_message "$YELLOW" "Warning: $STATISTIC_DIR/evaluation_results.csv not found"
        print_message "$YELLOW" "Running p_sim_score.py to generate similarity metrics..."
        cd "$SCORING_DIR"
        python p_sim_score.py
        if [ $? -ne 0 ]; then
            print_message "$RED" "Failed to run p_sim_score.py"
            missing_files=1
        else
            print_message "$GREEN" "Successfully generated similarity metrics"
        fi
    fi
    
    if [ $missing_files -eq 1 ]; then
        print_message "$RED" "Prerequisites check failed. Please ensure all required files exist."
        exit 1
    fi
    
    print_message "$GREEN" "All prerequisites satisfied"
}

# Create backup of original file
print_message "$YELLOW" "Creating backup of p_JLLM_score.py..."
cp "$JLLM_SCRIPT" "$BACKUP_FILE"

# Create output base directory if it doesn't exist
mkdir -p "$OUTPUT_BASE_DIR"

# Error tracking variables
ERROR_LOG="${OUTPUT_BASE_DIR}/error_report_$(date +%Y%m%d_%H%M%S).txt"
API_ERRORS=()
MISSING_DATA=()
FAILED_MODELS=()
SUCCESSFUL_MODELS=()

# Function to check and run evaluation for models missing evaluation_scores.csv
check_and_run_missing_evaluations() {
    local missing_models=()
    
    print_message "$YELLOW" "Checking for models with missing evaluation scores..."
    
    # Check each model directory for missing evaluation_scores.csv
    for model_dir in "$OUTPUT_LLMS_DIR"/*/; do
        if [ -d "$model_dir" ]; then
            model_name=$(basename "$model_dir")
            
            # Skip special directories
            if [[ "$model_name" == "combined_evaluation_scores"* ]] || [[ "$model_name" == "." ]] || [[ "$model_name" == ".." ]]; then
                continue
            fi
            
            # Check if model has response files but no evaluation_scores.csv
            has_responses=false
            has_evaluation=false
            
            # Check for response files in any subdirectory
            for subdir in "$model_dir"*/; do
                if [ -d "$subdir" ] && [ -f "$subdir/first_response.txt" ]; then
                    has_responses=true
                fi
                if [ -f "$subdir/evaluation_scores.csv" ]; then
                    has_evaluation=true
                fi
            done
            
            if [ "$has_responses" = true ] && [ "$has_evaluation" = false ]; then
                missing_models+=("$model_name")
                print_message "$YELLOW" "  Found model with missing evaluation: $model_name"
            fi
        fi
    done
    
    # If there are missing models, run evaluatePy.py for them
    if [ ${#missing_models[@]} -gt 0 ]; then
        print_message "$YELLOW" "\nFound ${#missing_models[@]} models needing evaluation"
        print_message "$YELLOW" "Running evaluatePy.py in headless mode..."
        
        # Check if xvfb is available
        if command -v xvfb-run &> /dev/null; then
            print_message "$GREEN" "Using xvfb-run for headless execution"
            HEADLESS_CMD="xvfb-run -a"
        else
            print_message "$YELLOW" "xvfb-run not found, using DISPLAY= instead"
            HEADLESS_CMD="env DISPLAY="
        fi
        
        # Update evaluatePy.py to include missing models
        cd "$SCORING_DIR"
        
        # Create a temporary evaluatePy script for missing models
        cp evaluatePy.py evaluatePy_temp.py
        
        # Update the test_model_list in the temporary script
        model_list_str=$(printf '"%s", ' "${missing_models[@]}")
        model_list_str="[${model_list_str%, }]"
        
        # Update the test_model_list line
        sed -i "83s/.*/test_model_list = $model_list_str/" evaluatePy_temp.py
        
        print_message "$YELLOW" "Running evaluation for missing models: ${missing_models[*]}"
        print_message "$YELLOW" "This may take a while without visual feedback..."
        
        # Run the evaluation script in headless mode
        $HEADLESS_CMD python evaluatePy_temp.py > "${OUTPUT_BASE_DIR}/missing_models_evaluation.log" 2>&1 &
        eval_pid=$!
        
        # Show progress indicator
        while kill -0 $eval_pid 2>/dev/null; do
            printf "  ⠋ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠙ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠹ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠸ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠼ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠴ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠦ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠧ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠇ Evaluating missing models...\r"
            sleep 0.5
            printf "  ⠏ Evaluating missing models...\r"
            sleep 0.5
        done
        
        wait $eval_pid
        eval_exit_code=$?
        printf "                                        \r"
        
        # Clean up temp file
        rm -f evaluatePy_temp.py
        
        if [ $eval_exit_code -eq 0 ]; then
            print_message "$GREEN" "✓ Successfully evaluated missing models"
            
            # Verify evaluation files were created
            for model in "${missing_models[@]}"; do
                eval_count=$(find "$OUTPUT_LLMS_DIR/$model" -name "evaluation_scores.csv" 2>/dev/null | wc -l)
                if [ $eval_count -gt 0 ]; then
                    print_message "$GREEN" "  ✓ $model: $eval_count evaluation files created"
                else
                    print_message "$YELLOW" "  ⚠ $model: No evaluation files found"
                fi
            done
        else
            print_message "$RED" "⚠ Evaluation failed for missing models"
            print_message "$YELLOW" "Check log at: ${OUTPUT_BASE_DIR}/missing_models_evaluation.log"
            print_message "$YELLOW" "Continuing with available evaluations..."
        fi
    else
        print_message "$GREEN" "All models have evaluation scores or no response files"
    fi
}

# Check prerequisites before starting
check_prerequisites

# Check and run evaluations for models with missing scores
check_and_run_missing_evaluations

# Save the original combined_evaluation_scores.csv if it exists
ORIGINAL_COMBINED_SCORES="${OUTPUT_LLMS_DIR}/combined_evaluation_scores_original.csv"
if [ -f "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" ] && [ ! -f "$ORIGINAL_COMBINED_SCORES" ]; then
    print_message "$YELLOW" "Backing up original combined_evaluation_scores.csv..."
    cp "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" "$ORIGINAL_COMBINED_SCORES"
fi

# Function to process a single judge model
process_judge_model() {
    local judge_model=$1
    local provider="${MODEL_PROVIDER[$judge_model]:-unknown}"
    
    if [[ "${FAILED_PROVIDERS[$provider]}" == "true" ]]; then
        print_message "$YELLOW" "\n⏭️ Skipping $judge_model - Provider '$provider' has exceeded failure limit"
        FAILED_MODELS+=("$judge_model: Skipped due to $provider API failures")
        return 1
    fi
    
    print_message "$GREEN" "\n=========================================="
    print_message "$GREEN" "Processing with judge model: $judge_model"
    print_message "$GREEN" "Provider: $provider"
    print_message "$GREEN" "=========================================="
    
    # Create output directory for this judge model
    model_output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"  # Replace dots with dashes for directory names
    mkdir -p "$model_output_dir"
    
    # Update the judge model in p_JLLM_score.py
    print_message "$YELLOW" "Updating judge model to $judge_model..."
    sed -i "587s/.*/evaluated_model = \"$judge_model\"/" "$JLLM_SCRIPT"
    
    # Verify the change
    if grep -q "evaluated_model = \"$judge_model\"" "$JLLM_SCRIPT"; then
        print_message "$GREEN" "Successfully updated judge model"
    else
        print_message "$RED" "Failed to update judge model"
        continue
    fi
    
    # Run the complete evaluation pipeline
    print_message "$YELLOW" "Running evaluation pipeline with judge: $judge_model"
    
    # Step 1: Run p_JLLM_score.py to generate scores with this judge
    print_message "$YELLOW" "Step 1: Running LLM-as-Judge scoring (p_JLLM_score.py)..."
    print_message "$YELLOW" "  Judge model: $judge_model"
    print_message "$YELLOW" "  Evaluating: Multiple S-LLM models across 34 simulation systems"
    
    cd "$SCORING_DIR/v01"
    
    # Run with error suppression and retry logic
    max_retries=3
    retry_count=0
    scoring_success=false
    timeout_duration=6000  # 60 seconds timeout per attempt
    
    while [ $retry_count -lt $max_retries ]; do
        retry_count=$((retry_count+1))
        print_message "$YELLOW" "  Attempt $retry_count/$max_retries for $provider API (timeout: ${timeout_duration}s)..."
        
        # Run in background with timeout to enable spinner and prevent hanging
        timeout $timeout_duration python p_JLLM_score.py > "${model_output_dir}/jllm_score_log.txt" 2>&1 &
        pid=$!
        
        # Monitor the process with spinner
        start_time=$(date +%s)
        while kill -0 $pid 2>/dev/null; do
            current_time=$(date +%s)
            elapsed=$((current_time - start_time))
            remaining=$((timeout_duration - elapsed))
            
            # Update spinner with time remaining
            printf " [⠋] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠙] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠹] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠸] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠼] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠴] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠦] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠧] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠇] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
            printf " [⠏] Evaluating with $judge_model (${elapsed}s elapsed, ${remaining}s remaining)...\r"
            sleep 0.5
        done
        
        wait $pid
        exit_code=$?
        printf "                                                                                \r"
        
        if [ $exit_code -eq 0 ]; then
            print_message "$GREEN" "✓ LLM-as-Judge scoring completed successfully"
            scoring_success=true
            SUCCESSFUL_MODELS+=("$judge_model")
            break
        elif [ $exit_code -eq 124 ]; then
            # Exit code 124 means timeout was reached
            print_message "$YELLOW" "  ⏱️ Timeout reached (${timeout_duration}s) for $judge_model"
            PROVIDER_FAILURES[$provider]=$((${PROVIDER_FAILURES[$provider]:-0} + 1))
            
            if [ $retry_count -lt $max_retries ]; then
                wait_time=$((20 * retry_count))  # 20s, 40s, 60s
                print_message "$YELLOW" "  Waiting ${wait_time}s before retry..."
                
                # Show countdown
                for ((i=wait_time; i>0; i--)); do
                    printf "  ⏳ Countdown: ${i}s remaining...\r"
                    sleep 1
                done
                printf "                                        \r"
            else
                print_message "$YELLOW" "⚠ Max attempts reached for $judge_model"
                FAILED_MODELS+=("$judge_model: Timeout after $max_retries attempts")
            fi
        else
            # Extract and display the actual error
            error_msg=$(tail -n 20 "${model_output_dir}/jllm_score_log.txt" | grep -i "error\|exception\|failed\|rate limit\|quota\|429\|insufficient" | head -1)
            if [ -z "$error_msg" ]; then
                error_msg=$(tail -n 5 "${model_output_dir}/jllm_score_log.txt" | tr '\n' ' ')
            fi
            
            if grep -qi "rate limit\|quota exceeded\|429\|insufficient_quota" "${model_output_dir}/jllm_score_log.txt"; then
                API_ERRORS+=("$judge_model: API rate limit/quota issue at attempt $retry_count")
                print_message "$YELLOW" "  ⚠️ API rate limit detected: ${error_msg:0:80}..."
                PROVIDER_FAILURES[$provider]=$((${PROVIDER_FAILURES[$provider]:-0} + 1))
                
                if [ $retry_count -lt $max_retries ]; then
                    wait_time=$((20 * retry_count))  # 20s, 40s, 60s
                    print_message "$YELLOW" "  Waiting ${wait_time}s before retry..."
                    
                    # Show countdown
                    for ((i=wait_time; i>0; i--)); do
                        printf "  ⏳ Countdown: ${i}s remaining...\r"
                        sleep 1
                    done
                    printf "                                        \r"
                else
                    print_message "$YELLOW" "⚠ Max retries reached for $judge_model"
                    FAILED_MODELS+=("$judge_model: API rate limit after $max_retries attempts")
                fi
            else
                # Check for other known errors and display them
                if grep -qi "connection\|timeout\|network" "${model_output_dir}/jllm_score_log.txt"; then
                    API_ERRORS+=("$judge_model: Network/connection issue")
                    print_message "$YELLOW" "  ⚠ Network issue: ${error_msg:0:80}..."
                elif grep -qi "file not found\|no such file" "${model_output_dir}/jllm_score_log.txt"; then
                    MISSING_DATA+=("$judge_model: Required input files missing")
                    missing_file=$(grep -i "file not found\|no such file" "${model_output_dir}/jllm_score_log.txt" | head -1)
                    print_message "$YELLOW" "  ⚠ Missing file: ${missing_file:0:80}..."
                else
                    FAILED_MODELS+=("$judge_model: Unknown error during scoring")
                    print_message "$YELLOW" "  ⚠ Error occurred: ${error_msg:0:80}..."
                fi
                print_message "$YELLOW" "  Check full log at: ${model_output_dir}/jllm_score_log.txt"
                print_message "$YELLOW" "⚠ Skipping $judge_model due to errors"
                break
            fi
        fi
    done
    
    # Check if provider has too many failures (3 or more)
    if [ "${PROVIDER_FAILURES[$provider]:-0}" -ge 3 ]; then
        FAILED_PROVIDERS[$provider]="true"
        print_message "$RED" "\n⛔ Provider '$provider' has failed 3+ times. Skipping all remaining $provider models."
        
        # Add all remaining models from this provider to failed list
        for remaining_model in "${JUDGE_MODELS[@]}"; do
            if [[ "${MODEL_PROVIDER[$remaining_model]}" == "$provider" ]] && [[ ! " ${SUCCESSFUL_MODELS[@]} " =~ " ${remaining_model} " ]] && [[ "$remaining_model" != "$judge_model" ]]; then
                FAILED_MODELS+=("$remaining_model: Auto-skipped due to $provider API failures")
            fi
        done
    fi
    
    if [ "$scoring_success" = false ]; then
        continue
    fi
    
    # Check if combined_evaluation_scores.csv was created
    if [ ! -f "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" ]; then
        MISSING_DATA+=("$judge_model: combined_evaluation_scores.csv not generated")
        print_message "$YELLOW" "⚠ Skipping $judge_model due to missing output data"
        continue
    fi
    
    # Copy the judge-specific combined scores to the model output directory
    cp "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" "${model_output_dir}/combined_evaluation_scores_${judge_model//\./-}.csv"
    
    # Step 2: Check if similarity metrics exist (should already be there)
    print_message "$YELLOW" "Step 2: Checking similarity metrics..."
    if [ -f "$STATISTIC_DIR/evaluation_results.csv" ]; then
        print_message "$GREEN" "Similarity metrics found (evaluation_results.csv)"
        cp "$STATISTIC_DIR/evaluation_results.csv" "${model_output_dir}/evaluation_results.csv"
    else
        print_message "$YELLOW" "Similarity metrics not found, generating..."
        cd "$SCORING_DIR"
        python p_sim_score.py 2>&1 | tee "${model_output_dir}/sim_score_log.txt"
        
        if [ ${PIPESTATUS[0]} -eq 0 ] && [ -f "$STATISTIC_DIR/evaluation_results.csv" ]; then
            print_message "$GREEN" "Similarity scoring completed successfully"
            cp "$STATISTIC_DIR/evaluation_results.csv" "${model_output_dir}/evaluation_results.csv"
        else
            print_message "$RED" "Similarity scoring failed"
            print_message "$YELLOW" "Rankings will be based on LLM-as-Judge scores only"
        fi
    fi
    
    # Step 3: Run rank_llm.py to generate rankings
    print_message "$YELLOW" "Step 3: Generating rankings (rank_llm.py)..."
    cd "$SCORING_DIR"
    
    # Temporarily update rank_llm.py to use the judge-specific output directory
    # First, let's check current output directory in rank_llm.py
    if grep -q "OUT_DIR = Path" "$SCORING_DIR/rank_llm.py"; then
        # Backup rank_llm.py
        cp "$SCORING_DIR/rank_llm.py" "$SCORING_DIR/rank_llm.py.backup"
        
        # Update output directory for this judge
        sed -i "s|OUT_DIR = Path.*|OUT_DIR = Path(\"${model_output_dir}\")|" "$SCORING_DIR/rank_llm.py"
    fi
    
    python rank_llm.py 2>&1 | tee "${model_output_dir}/rank_llm_log.txt"
    
    # Check if ranking completed successfully
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        print_message "$GREEN" "Ranking completed successfully"
    else
        print_message "$RED" "Ranking failed"
        print_message "$YELLOW" "Check log at ${model_output_dir}/rank_llm_log.txt"
    fi
    
    # Restore rank_llm.py if we modified it
    if [ -f "$SCORING_DIR/rank_llm.py.backup" ]; then
        mv "$SCORING_DIR/rank_llm.py.backup" "$SCORING_DIR/rank_llm.py"
    fi
    
    # Copy any additional output files from the default out directory if they were created there
    if [ -d "${SCORING_DIR}/out" ]; then
        print_message "$YELLOW" "Copying additional output files..."
        for file in "${SCORING_DIR}/out/"*; do
            if [ -f "$file" ]; then
                filename=$(basename "$file")
                if [ ! -f "${model_output_dir}/$filename" ]; then
                    cp "$file" "${model_output_dir}/"
                fi
            fi
        done
    fi
    
    # Add judge model information to the consensus rankings file
    if [ -f "${model_output_dir}/consensus_llm_rankings.csv" ]; then
        # Add a column indicating which judge model was used
        sed -i "1s/$/,Judge_Model/" "${model_output_dir}/consensus_llm_rankings.csv"
        sed -i "2,\$s/$/,${judge_model}/" "${model_output_dir}/consensus_llm_rankings.csv"
    fi
    
    print_message "$GREEN" "Completed evaluation with judge model: $judge_model"
    print_message "$GREEN" "Results saved to: ${model_output_dir}"
    
    # List the key output files
    print_message "$YELLOW" "\nKey output files generated:"
    [ -f "${model_output_dir}/consensus_llm_rankings.csv" ] && echo "  - consensus_llm_rankings.csv"
    [ -f "${model_output_dir}/llm_all_metrics_with_rank.csv" ] && echo "  - llm_all_metrics_with_rank.csv"
    [ -f "${model_output_dir}/combined_evaluation_scores_${judge_model//\./-}.csv" ] && echo "  - combined_evaluation_scores_${judge_model//\./-}.csv"
    [ -f "${model_output_dir}/evaluation_results.csv" ] && echo "  - evaluation_results.csv"
    [ -f "${model_output_dir}/consensus_llm_rankings_top10.png" ] && echo "  - consensus_llm_rankings_top10.png"
}

# Separate OpenAI models for parallel processing
OPENAI_JUDGES=()
OTHER_JUDGES=()

for model in "${JUDGE_MODELS[@]}"; do
    provider="${MODEL_PROVIDER[$model]:-unknown}"
    if [[ "$provider" == "openai" ]]; then
        OPENAI_JUDGES+=("$model")
    else
        OTHER_JUDGES+=("$model")
    fi
done

print_message "$YELLOW" "\n========================================="
print_message "$YELLOW" "Processing ${#OPENAI_JUDGES[@]} OpenAI models in parallel..."
print_message "$YELLOW" "========================================="

# Process OpenAI models in parallel (they can handle concurrent requests)
if [ ${#OPENAI_JUDGES[@]} -gt 0 ]; then
    for judge_model in "${OPENAI_JUDGES[@]}"; do
        (
            process_judge_model "$judge_model"
        ) &
    done
    
    # Wait for all OpenAI models to complete
    wait
    print_message "$GREEN" "\n✓ All OpenAI judge models completed"
fi

# Process other models sequentially (to avoid rate limits)
if [ ${#OTHER_JUDGES[@]} -gt 0 ]; then
    print_message "$YELLOW" "\n========================================="
    print_message "$YELLOW" "Processing ${#OTHER_JUDGES[@]} non-OpenAI models sequentially..."
    print_message "$YELLOW" "========================================="
    
    for judge_model in "${OTHER_JUDGES[@]}"; do
        process_judge_model "$judge_model"
    done
fi

# Restore original p_JLLM_score.py
print_message "$YELLOW" "\nRestoring original p_JLLM_score.py..."
mv "$BACKUP_FILE" "$JLLM_SCRIPT"

# Restore original combined_evaluation_scores.csv if we backed it up
if [ -f "$ORIGINAL_COMBINED_SCORES" ]; then
    print_message "$YELLOW" "Restoring original combined_evaluation_scores.csv..."
    mv "$ORIGINAL_COMBINED_SCORES" "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv"
fi

# Generate comparative summary report
print_message "$YELLOW" "\nGenerating comparative summary report..."
SUMMARY_FILE="${OUTPUT_BASE_DIR}/summary_rankings_comparison.txt"
COMPARISON_CSV="${OUTPUT_BASE_DIR}/top10_comparison.csv"

echo "Summary of Rankings with Different Judge Models" > "$SUMMARY_FILE"
echo "================================================" >> "$SUMMARY_FILE"
echo "Generated on: $(date)" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

# Create CSV header for comparison
echo "Rank,$(IFS=,; echo "${JUDGE_MODELS[*]}")" > "$COMPARISON_CSV"

# Collect top 10 for each judge
for rank in {1..10}; do
    row="$rank"
    for judge_model in "${JUDGE_MODELS[@]}"; do
        model_output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"
        consensus_file="${model_output_dir}/consensus_llm_rankings.csv"
        
        if [ -f "$consensus_file" ]; then
            # Get the model at this rank (skip header, get line at rank position)
            model_at_rank=$(awk -F',' "NR==$((rank+1)) {print \$2}" "$consensus_file" 2>/dev/null)
            row="${row},${model_at_rank:-N/A}"
        else
            row="${row},N/A"
        fi
    done
    echo "$row" >> "$COMPARISON_CSV"
done

# Add detailed rankings to summary
for judge_model in "${JUDGE_MODELS[@]}"; do
    model_output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"
    consensus_file="${model_output_dir}/consensus_llm_rankings.csv"
    
    echo "==========================================" >> "$SUMMARY_FILE"
    echo "Judge Model: $judge_model" >> "$SUMMARY_FILE"
    echo "==========================================" >> "$SUMMARY_FILE"
    
    if [ -f "$consensus_file" ]; then
        echo "Top 10 Rankings:" >> "$SUMMARY_FILE"
        head -n 11 "$consensus_file" | tail -n 10 | awk -F',' '{printf "  %2d. %-30s Score: %s\n", $1, $2, $3}' >> "$SUMMARY_FILE"
    else
        echo "  No rankings found" >> "$SUMMARY_FILE"
    fi
    echo "" >> "$SUMMARY_FILE"
done

print_message "$GREEN" "\n=========================================="
print_message "$GREEN" "All evaluations completed!"
print_message "$GREEN" "=========================================="
print_message "$GREEN" "Results stored in: $OUTPUT_BASE_DIR"
print_message "$GREEN" "Summary report: ${SUMMARY_FILE}"
print_message "$GREEN" "Comparison CSV: ${COMPARISON_CSV}"
print_message "$GREEN" "=========================================="

# Display directory structure
print_message "$YELLOW" "\nOutput directory structure:"
if command -v tree &> /dev/null; then
    tree -L 2 "$OUTPUT_BASE_DIR" 2>/dev/null
else
    ls -la "$OUTPUT_BASE_DIR"
    for dir in "$OUTPUT_BASE_DIR"/*/; do
        echo "  $(basename $dir)/"
        ls -la "$dir" | head -10
    done
fi

print_message "$YELLOW" "\nTop 10 Comparison across judges:"
cat "$COMPARISON_CSV" | column -t -s ','

# Generate error report function
generate_error_report() {
    echo "========================================" > "$ERROR_LOG"
    echo "LLM Judge Evaluation Error Report" >> "$ERROR_LOG"
    echo "Generated: $(date)" >> "$ERROR_LOG"
    echo "========================================" >> "$ERROR_LOG"
    
    echo -e "\n## Summary Statistics:" >> "$ERROR_LOG"
    echo "  Total models attempted: ${#JUDGE_MODELS[@]}" >> "$ERROR_LOG"
    echo "  Successful: ${#SUCCESSFUL_MODELS[@]}" >> "$ERROR_LOG"
    echo "  Failed: ${#FAILED_MODELS[@]}" >> "$ERROR_LOG"
    echo "" >> "$ERROR_LOG"
    
    echo "## Successful Models:" >> "$ERROR_LOG"
    if [ ${#SUCCESSFUL_MODELS[@]} -eq 0 ]; then
        echo "  None" >> "$ERROR_LOG"
    else
        for model in "${SUCCESSFUL_MODELS[@]}"; do
            echo "  ✓ $model" >> "$ERROR_LOG"
        done
    fi
    echo "" >> "$ERROR_LOG"
    
    echo "## Failed API Providers:" >> "$ERROR_LOG"
    if [ ${#FAILED_PROVIDERS[@]} -eq 0 ]; then
        echo "  None" >> "$ERROR_LOG"
    else
        for provider in "${!FAILED_PROVIDERS[@]}"; do
            failures="${PROVIDER_FAILURES[$provider]}"
            echo "  ⛔ $provider (${failures} failures) - All models skipped" >> "$ERROR_LOG"
        done
    fi
    echo "" >> "$ERROR_LOG"
    
    echo "## API Issues Encountered:" >> "$ERROR_LOG"
    if [ ${#API_ERRORS[@]} -eq 0 ]; then
        echo "  None" >> "$ERROR_LOG"
    else
        for error in "${API_ERRORS[@]}"; do
            echo "  ⚠ $error" >> "$ERROR_LOG"
        done
    fi
    echo "" >> "$ERROR_LOG"
    
    echo "## Missing Data Issues:" >> "$ERROR_LOG"
    if [ ${#MISSING_DATA[@]} -eq 0 ]; then
        echo "  None" >> "$ERROR_LOG"
    else
        for error in "${MISSING_DATA[@]}"; do
            echo "  📁 $error" >> "$ERROR_LOG"
        done
    fi
    echo "" >> "$ERROR_LOG"
    
    echo "## Failed Models (Other Errors):" >> "$ERROR_LOG"
    if [ ${#FAILED_MODELS[@]} -eq 0 ]; then
        echo "  None" >> "$ERROR_LOG"
    else
        for error in "${FAILED_MODELS[@]}"; do
            echo "  ✗ $error" >> "$ERROR_LOG"
        done
    fi
    echo "" >> "$ERROR_LOG"
    
    echo "## Recommendations:" >> "$ERROR_LOG"
    if [ ${#API_ERRORS[@]} -gt 0 ]; then
        echo "  • API Issues: Check API keys and rate limits" >> "$ERROR_LOG"
        echo "    - Verify API keys are correctly set in .env file" >> "$ERROR_LOG"
        echo "    - Check account quotas and billing status" >> "$ERROR_LOG"
        echo "    - Consider adding delays between API calls" >> "$ERROR_LOG"
    fi
    if [ ${#MISSING_DATA[@]} -gt 0 ]; then
        echo "  • Missing Data: Ensure all required input files exist" >> "$ERROR_LOG"
        echo "    - Run p_sim_score.py to generate evaluation_results.csv" >> "$ERROR_LOG"
        echo "    - Verify output_llms/ directory contains model outputs" >> "$ERROR_LOG"
        echo "    - Check demo_data/ for ground truth files" >> "$ERROR_LOG"
    fi
    if [ ${#FAILED_MODELS[@]} -gt 0 ]; then
        echo "  • Failed Models: Review individual log files in out_diff_models/" >> "$ERROR_LOG"
        echo "    - Check jllm_score_log.txt for detailed error messages" >> "$ERROR_LOG"
    fi
    
    # Print summary to console
    print_message "$YELLOW" "\n📋 Error Report Summary:"
    print_message "$GREEN" "  Successful: ${#SUCCESSFUL_MODELS[@]}/${#JUDGE_MODELS[@]} models"
    
    if [ ${#API_ERRORS[@]} -gt 0 ]; then
        print_message "$YELLOW" "  API Issues: ${#API_ERRORS[@]} occurrences"
    fi
    if [ ${#MISSING_DATA[@]} -gt 0 ]; then
        print_message "$YELLOW" "  Missing Data: ${#MISSING_DATA[@]} models affected"
    fi
    if [ ${#FAILED_MODELS[@]} -gt 0 ]; then
        print_message "$RED" "  Failed: ${#FAILED_MODELS[@]} models"
    fi
    
    print_message "$YELLOW" "\n📄 Detailed error report saved to: $ERROR_LOG"
}

# Generate the error report
generate_error_report