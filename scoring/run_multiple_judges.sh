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

# Export API Keys for all providers
export OPENAI_API_KEY=""
export ANTHROPIC_API_KEY=""
export GOOGLE_API_KEY=""
export MISTRAL_API_KEY=""
export NVIDIA_API_KEY=""
export GEMINI_API_KEY="${GOOGLE_API_KEY}"  # Google Gemini uses the same key

# Define judge models to test
# Including all available models from p_JLLM_score.py
JUDGE_MODELS=(
    # OpenAI Models
    "gpt-4o"
    "gpt-4o-mini"
    "gpt-4.1"
    "gpt-4.1-mini"
    "gpt-4.1-nano"
    "o3"
    "o4-mini"
    
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

# Check prerequisites before starting
check_prerequisites

# Save the original combined_evaluation_scores.csv if it exists
ORIGINAL_COMBINED_SCORES="${OUTPUT_LLMS_DIR}/combined_evaluation_scores_original.csv"
if [ -f "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" ] && [ ! -f "$ORIGINAL_COMBINED_SCORES" ]; then
    print_message "$YELLOW" "Backing up original combined_evaluation_scores.csv..."
    cp "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" "$ORIGINAL_COMBINED_SCORES"
fi

# Main loop through judge models
for judge_model in "${JUDGE_MODELS[@]}"; do
    print_message "$GREEN" "\n=========================================="
    print_message "$GREEN" "Processing with judge model: $judge_model"
    print_message "$GREEN" "=========================================="
    
    # Create output directory for this judge model
    model_output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"  # Replace dots with dashes for directory names
    mkdir -p "$model_output_dir"
    
    # Update the judge model in p_JLLM_score.py
    print_message "$YELLOW" "Updating judge model to $judge_model..."
    sed -i "393s/.*/evaluated_model = \"$judge_model\"/" "$JLLM_SCRIPT"
    
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
    print_message "$YELLOW" "This will evaluate all test models and systems using $judge_model as judge"
    
    cd "$SCORING_DIR/v01"
    python p_JLLM_score.py 2>&1 | tee "${model_output_dir}/jllm_score_log.txt"
    
    # Check if scoring completed successfully
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        print_message "$GREEN" "LLM-as-Judge scoring completed successfully"
    else
        print_message "$RED" "LLM-as-Judge scoring failed for $judge_model"
        print_message "$YELLOW" "Check log at ${model_output_dir}/jllm_score_log.txt"
        continue
    fi
    
    # Check if combined_evaluation_scores.csv was created
    if [ ! -f "${OUTPUT_LLMS_DIR}/combined_evaluation_scores.csv" ]; then
        print_message "$RED" "Error: combined_evaluation_scores.csv was not generated"
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
done

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