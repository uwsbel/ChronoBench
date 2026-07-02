#!/bin/bash
#SBATCH --job-name=dtgen-array
#SBATCH --partition=batch
#SBATCH --qos=interactive
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gres=gpu:4
#SBATCH --cpus-per-task=32
#SBATCH --mem=0
#SBATCH --time=04:00:00
#SBATCH --array=1-1%1
#SBATCH --output=logs/dtgen_array_%A_%a.out
#SBATCH --error=logs/dtgen_array_%A_%a.err

set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/lustre/fsw/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/benchmarks/SimBench}"
MODEL_LIST="${MODEL_LIST:-$PROJECT_ROOT/scoring/v01/models.tsv}"
RUN_SCRIPT="${RUN_SCRIPT:-$PROJECT_ROOT/scoring/v01/run_simbench_sft32b.sbatch}"

cd "$PROJECT_ROOT"
mkdir -p logs

if [[ -z "${SLURM_ARRAY_TASK_ID:-}" ]]; then
  echo "SLURM_ARRAY_TASK_ID is required. Submit with sbatch --array=1-N%K." >&2
  exit 1
fi

if [[ ! -f "$MODEL_LIST" ]]; then
  echo "Model list not found: $MODEL_LIST" >&2
  exit 1
fi

MODEL_LINE="$(
  awk -v target="$SLURM_ARRAY_TASK_ID" '
    BEGIN { FS = "\t" }
    /^[[:space:]]*($|#)/ { next }
    {
      row += 1
      if (row == target) {
        print
        exit
      }
    }
  ' "$MODEL_LIST"
)"

if [[ -z "$MODEL_LINE" ]]; then
  echo "No model entry for array task $SLURM_ARRAY_TASK_ID in $MODEL_LIST" >&2
  exit 1
fi

field() {
  awk -v idx="$1" 'BEGIN { FS = "\t" } { print $idx }' <<< "$MODEL_LINE"
}

MODEL_NAME_FIELD="$(field 1)"
MODEL_PATH_FIELD="$(field 2)"
TENSOR_PARALLEL_SIZE_FIELD="$(field 3)"
MAX_WORKERS_FIELD="$(field 4)"
GPU_MEMORY_UTILIZATION_FIELD="$(field 5)"
MAX_MODEL_LEN_FIELD="$(field 6)"
SYSTEMS_FIELD="$(field 7)"
LIMIT_FIELD="$(field 8)"
REASONING_PARSER_FIELD="$(field 9)"
CHAT_TEMPLATE_KWARGS_JSON_FIELD="$(field 10)"

if [[ -z "$MODEL_NAME_FIELD" || -z "$MODEL_PATH_FIELD" ]]; then
  echo "Model list row $SLURM_ARRAY_TASK_ID must define model_name and model_path." >&2
  exit 1
fi

export MODEL_NAME="$MODEL_NAME_FIELD"
export MODEL_PATH="$MODEL_PATH_FIELD"

if [[ -n "$TENSOR_PARALLEL_SIZE_FIELD" ]]; then
  export TENSOR_PARALLEL_SIZE="$TENSOR_PARALLEL_SIZE_FIELD"
fi

if [[ -n "$MAX_WORKERS_FIELD" ]]; then
  export MAX_WORKERS="$MAX_WORKERS_FIELD"
fi

if [[ -n "$GPU_MEMORY_UTILIZATION_FIELD" ]]; then
  export GPU_MEMORY_UTILIZATION="$GPU_MEMORY_UTILIZATION_FIELD"
fi

if [[ -n "$MAX_MODEL_LEN_FIELD" ]]; then
  export MAX_MODEL_LEN="$MAX_MODEL_LEN_FIELD"
fi

if [[ -n "$SYSTEMS_FIELD" ]]; then
  export SYSTEMS="$SYSTEMS_FIELD"
fi

if [[ -n "$LIMIT_FIELD" ]]; then
  export LIMIT="$LIMIT_FIELD"
fi

if [[ -n "$REASONING_PARSER_FIELD" ]]; then
  export REASONING_PARSER="$REASONING_PARSER_FIELD"
fi

if [[ -n "$CHAT_TEMPLATE_KWARGS_JSON_FIELD" ]]; then
  export CHAT_TEMPLATE_KWARGS_JSON="$CHAT_TEMPLATE_KWARGS_JSON_FIELD"
fi

echo "Array task $SLURM_ARRAY_TASK_ID generating $MODEL_NAME"
bash "$RUN_SCRIPT"
