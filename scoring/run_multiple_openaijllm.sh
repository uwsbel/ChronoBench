#!/bin/bash

# Enhanced script to run 3 OpenAI judge LLMs in parallel with proper logging
# Uses separate API keys for each judge model to avoid rate limit conflicts

# Set working directory
SCORING_DIR="/home/hongyu/Documents/SimBench/scoring"
OUTPUT_BASE_DIR="${SCORING_DIR}/out_diff_models"
SCRIPTS_DIR="${SCORING_DIR}/v01"
OUTPUT_LLMS_DIR="/home/hongyu/Documents/SimBench/output_llms"

# Load API keys from .env file
ENV_FILE="${SCORING_DIR}/../.env"
if [ -f "$ENV_FILE" ]; then
    set -a  # automatically export all variables
    source "$ENV_FILE"
    set +a
    echo "✓ API keys loaded from .env file"
else
    echo "⚠ Warning: .env file not found at $ENV_FILE"
    exit 1
fi

# Export API keys for each model
export OPENAI_API_KEY_1="${OPENAI_API_KEY_1:-$OPENAI_API_KEY}"
export OPENAI_API_KEY_2="${OPENAI_API_KEY_2:-$OPENAI_API_KEY}"
export OPENAI_API_KEY_3="${OPENAI_API_KEY_3:-$OPENAI_API_KEY}"
export NVIDIA_API_KEY="${NVIDIA_API_KEY:-}"

# The three OpenAI judge models and their corresponding scripts
declare -A JUDGE_SCRIPTS=(
    ["gpt-4o-mini"]="p_JLLM_score_gpt4omini.py"
    ["gpt-4.1-mini"]="p_JLLM_score_gpt41mini.py"
    ["gpt-4.1-nano"]="p_JLLM_score_gpt41nano.py"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to monitor a log file
monitor_log() {
    local log_file=$1
    local model=$2
    
    if [ -f "$log_file" ]; then
        # Get last 5 lines that show progress
        local last_lines=$(tail -5 "$log_file" | grep -E "✓|✗|Processing|Rate limit|Waiting" | tail -3)
        if [ -n "$last_lines" ]; then
            echo "$last_lines" | while IFS= read -r line; do
                print_message "$BLUE" "  [$model] $line"
            done
        fi
    fi
}

# Function to check if a process is still running
check_process() {
    local pid=$1
    if kill -0 $pid 2>/dev/null; then
        return 0  # Process is running
    else
        return 1  # Process is not running
    fi
}

# Main execution
print_message "$GREEN" "\n=========================================="
print_message "$GREEN" "Parallel OpenAI Judge LLM Evaluation"
print_message "$GREEN" "=========================================="
print_message "$YELLOW" "Judge models:"
print_message "$YELLOW" "  1. gpt-4o-mini  (using OPENAI_API_KEY_1)"
print_message "$YELLOW" "  2. gpt-4.1-mini (using OPENAI_API_KEY_2)"
print_message "$YELLOW" "  3. gpt-4.1-nano (using OPENAI_API_KEY_3)"
print_message "$GREEN" "=========================================="

# Create output directories
mkdir -p "$OUTPUT_BASE_DIR"

# Array to store background process PIDs
declare -A PIDS
declare -A LOG_FILES

# Start all three judge models in parallel
print_message "$YELLOW" "\nLaunching judge models in parallel..."

for judge_model in "${!JUDGE_SCRIPTS[@]}"; do
    script="${JUDGE_SCRIPTS[$judge_model]}"
    script_path="${SCRIPTS_DIR}/$script"
    output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"
    log_file="${output_dir}/jllm_score_log.txt"
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Check if script exists
    if [ ! -f "$script_path" ]; then
        print_message "$RED" "✗ Script not found: $script_path"
        continue
    fi
    
    print_message "$YELLOW" "  • Starting $judge_model with $script"
    print_message "$YELLOW" "    Log: $log_file"
    
    # Run the script in background
    cd "$SCRIPTS_DIR"
    nohup python "$script" >> "$log_file" 2>&1 &
    pid=$!
    
    PIDS["$judge_model"]=$pid
    LOG_FILES["$judge_model"]=$log_file
    
    print_message "$GREEN" "    ✓ Started with PID: $pid"
done

print_message "$GREEN" "\n✓ All judge models launched successfully!"
print_message "$YELLOW" "=========================================="

# Monitor the processes
print_message "$YELLOW" "\nMonitoring progress (press Ctrl+C to stop monitoring)..."
print_message "$YELLOW" "Note: The scripts will continue running even if you stop monitoring.\n"

# Trap to handle Ctrl+C gracefully
trap 'print_message "$YELLOW" "\nStopped monitoring. Processes are still running in background."; exit 0' INT

# Monitor loop
while true; do
    all_done=true
    
    # Clear previous output (optional, comment out if you want to see history)
    # clear
    
    print_message "$GREEN" "\n===== STATUS UPDATE $(date '+%Y-%m-%d %H:%M:%S') ====="
    
    for judge_model in "${!PIDS[@]}"; do
        pid=${PIDS[$judge_model]}
        log_file=${LOG_FILES[$judge_model]}
        
        print_message "$YELLOW" "\n[$judge_model] PID: $pid"
        
        if check_process $pid; then
            print_message "$GREEN" "  Status: RUNNING ✓"
            monitor_log "$log_file" "$judge_model"
            all_done=false
        else
            # Check exit status
            wait $pid 2>/dev/null
            exit_code=$?
            if [ $exit_code -eq 0 ]; then
                print_message "$GREEN" "  Status: COMPLETED ✓"
            else
                print_message "$RED" "  Status: FAILED (exit code: $exit_code) ✗"
            fi
        fi
    done
    
    # If all processes are done, exit the monitor loop
    if [ "$all_done" = true ]; then
        print_message "$GREEN" "\n=========================================="
        print_message "$GREEN" "All evaluations completed!"
        print_message "$GREEN" "=========================================="
        break
    fi
    
    # Check log file sizes and last update times
    print_message "$YELLOW" "\n--- Log File Status ---"
    for judge_model in "${!LOG_FILES[@]}"; do
        log_file=${LOG_FILES[$judge_model]}
        if [ -f "$log_file" ]; then
            size=$(du -h "$log_file" | cut -f1)
            last_mod=$(stat -c %y "$log_file" | cut -d' ' -f2 | cut -d'.' -f1)
            print_message "$BLUE" "  [$judge_model] Size: $size, Last update: $last_mod"
        fi
    done
    
    # Wait before next update
    sleep 30
done

# Generate final summary
print_message "$YELLOW" "\n--- Final Summary ---"
for judge_model in "${!LOG_FILES[@]}"; do
    log_file=${LOG_FILES[$judge_model]}
    output_dir="${OUTPUT_BASE_DIR}/out_${judge_model//\./-}"
    
    print_message "$YELLOW" "\n$judge_model:"
    
    if [ -f "$log_file" ]; then
        # Count completed evaluations
        completed=$(grep -c "✓ Completed:" "$log_file" 2>/dev/null || echo "0")
        failed=$(grep -c "✗ Failed:" "$log_file" 2>/dev/null || echo "0")
        print_message "$GREEN" "  Completed: $completed"
        print_message "$RED" "  Failed: $failed"
    fi
    
    # Check for output files
    if [ -d "$output_dir" ]; then
        csv_count=$(find "${OUTPUT_LLMS_DIR}" -name "evaluation_scores.csv" | wc -l)
        print_message "$BLUE" "  CSV files generated: $csv_count"
    fi
done

print_message "$GREEN" "\n=========================================="
print_message "$GREEN" "Evaluation process complete!"
print_message "$GREEN" "Results stored in: $OUTPUT_BASE_DIR"
print_message "$GREEN" "=========================================="

# Optional: Show how to view logs
print_message "$YELLOW" "\nTo view logs in real-time, use:"
print_message "$YELLOW" "  tail -f ${OUTPUT_BASE_DIR}/out_gpt-4o-mini/jllm_score_log.txt"
print_message "$YELLOW" "  tail -f ${OUTPUT_BASE_DIR}/out_gpt-4-1-mini/jllm_score_log.txt"
print_message "$YELLOW" "  tail -f ${OUTPUT_BASE_DIR}/out_gpt-4-1-nano/jllm_score_log.txt"