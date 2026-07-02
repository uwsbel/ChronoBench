#!/bin/bash
#SBATCH --job-name=dtjudge
#SBATCH --partition=batch
#SBATCH --qos=interactive
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=16
#SBATCH --mem=64G
#SBATCH --time=04:00:00
#SBATCH --output=logs/dtjudge_%j.out
#SBATCH --error=logs/dtjudge_%j.err

set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/lustre/fsw/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/benchmarks/SimBench}"
CONDA_ROOT="${CONDA_ROOT:-/lustre/fs1/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/miniforge3}"
MODEL_NAME="${MODEL_NAME:-sft-task-32b-lr2x-3156662}"
JUDGE_MODEL="${JUDGE_MODEL:-gpt-4o-mini}"
JUDGE_MODES="${JUDGE_MODES:-reference_document}"
MAX_WORKERS="${MAX_WORKERS:-8}"

cd "$PROJECT_ROOT"
mkdir -p logs

if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  echo "OPENAI_API_KEY is required for OpenAI judge scoring." >&2
  exit 1
fi

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate vllm

python "$PROJECT_ROOT/scoring/v01/score_jllm_openai.py" \
  --model-name "$MODEL_NAME" \
  --judge-model "$JUDGE_MODEL" \
  --modes "$JUDGE_MODES" \
  --max-workers "$MAX_WORKERS"
