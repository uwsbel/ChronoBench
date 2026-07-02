#!/bin/bash
#SBATCH --job-name=dtgen-sft32b
#SBATCH --partition=batch
#SBATCH --qos=interactive
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gres=gpu:4
#SBATCH --cpus-per-task=32
#SBATCH --mem=0
#SBATCH --time=04:00:00
#SBATCH --output=logs/dtgen_sft32b_%j.out
#SBATCH --error=logs/dtgen_sft32b_%j.err

set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/lustre/fsw/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/benchmarks/SimBench}"
CONDA_ROOT="${CONDA_ROOT:-/lustre/fs1/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/miniforge3}"
MODEL_PATH="${MODEL_PATH:-/lustre/fsw/portfolios/coreai/projects/coreai_nvfm_cupilot/users/jingquanw/Automodel/checkpoints/sft-task-32b-lr2x/3156662/LATEST/model/consolidated}"
MODEL_NAME="${MODEL_NAME:-sft-task-32b-lr2x-3156662}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8000}"
TENSOR_PARALLEL_SIZE="${TENSOR_PARALLEL_SIZE:-4}"
MAX_MODEL_LEN="${MAX_MODEL_LEN:-32768}"
GPU_MEMORY_UTILIZATION="${GPU_MEMORY_UTILIZATION:-0.85}"
MAX_WORKERS="${MAX_WORKERS:-8}"
MAX_TOKENS="${MAX_TOKENS:-4096}"
SYSTEMS="${SYSTEMS:-}"
SYSTEMS_FILE="${SYSTEMS_FILE:-}"
LIMIT="${LIMIT:-}"
CHAT_TEMPLATE_KWARGS_JSON="${CHAT_TEMPLATE_KWARGS_JSON:-}"
REASONING_PARSER="${REASONING_PARSER:-qwen3}"

cd "$PROJECT_ROOT"
mkdir -p logs

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate vllm

export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_DATASETS_OFFLINE="${HF_DATASETS_OFFLINE:-1}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export TOKENIZERS_PARALLELISM="${TOKENIZERS_PARALLELISM:-false}"
export VLLM_USE_DEEP_GEMM=0
export VLLM_MOE_USE_DEEP_GEMM=0
export VLLM_USE_DEEP_GEMM_E8M0=0
export VLLM_DEEP_GEMM_WARMUP=skip

VLLM_LOG="$PROJECT_ROOT/logs/vllm_${MODEL_NAME}_${SLURM_JOB_ID:-manual}.log"

python -m vllm.entrypoints.openai.api_server \
  --model "$MODEL_PATH" \
  --served-model-name "$MODEL_NAME" \
  --trust-remote-code \
  --dtype bfloat16 \
  --tensor-parallel-size "$TENSOR_PARALLEL_SIZE" \
  --max-model-len "$MAX_MODEL_LEN" \
  --gpu-memory-utilization "$GPU_MEMORY_UTILIZATION" \
  --reasoning-parser "$REASONING_PARSER" \
  --host "$HOST" \
  --port "$PORT" \
  > "$VLLM_LOG" 2>&1 &

SERVER_PID=$!

cleanup() {
  kill "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

python "$PROJECT_ROOT/scoring/v01/wait_openai_server.py" \
  --base-url "http://${HOST}:${PORT}/v1" \
  --timeout 1800 \
  --interval 10 &
WAIT_PID=$!

while kill -0 "$WAIT_PID" 2>/dev/null; do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    wait "$SERVER_PID" 2>/dev/null || true
    wait "$WAIT_PID" 2>/dev/null || true
    echo "vLLM server exited before endpoint became ready. See $VLLM_LOG" >&2
    exit 1
  fi
  sleep 5
done

wait "$WAIT_PID"

GEN_ARGS=(
  --model-name "$MODEL_NAME"
  --model-id "$MODEL_NAME"
  --base-url "http://${HOST}:${PORT}/v1"
  --max-workers "$MAX_WORKERS"
  --max-tokens "$MAX_TOKENS"
  --skip-existing
)

if [[ -z "$CHAT_TEMPLATE_KWARGS_JSON" ]]; then
  CHAT_TEMPLATE_KWARGS_JSON='{"enable_thinking": true}'
fi

if [[ -n "$SYSTEMS_FILE" ]]; then
  if [[ "$SYSTEMS_FILE" != /* ]]; then
    SYSTEMS_FILE="$PROJECT_ROOT/$SYSTEMS_FILE"
  fi
  if [[ ! -f "$SYSTEMS_FILE" ]]; then
    echo "SYSTEMS_FILE does not exist: $SYSTEMS_FILE" >&2
    exit 1
  fi

  SYSTEMS=""
  while IFS= read -r system_name || [[ -n "$system_name" ]]; do
    system_name="${system_name%%#*}"
    system_name="${system_name//[[:space:]]/}"
    [[ -z "$system_name" ]] && continue
    if [[ -n "$SYSTEMS" ]]; then
      SYSTEMS+=",${system_name}"
    else
      SYSTEMS="$system_name"
    fi
  done < "$SYSTEMS_FILE"
fi

if [[ -n "$SYSTEMS" ]]; then
  GEN_ARGS+=(--systems "$SYSTEMS")
fi

if [[ -n "$LIMIT" ]]; then
  GEN_ARGS+=(--limit "$LIMIT")
fi

if [[ -n "$CHAT_TEMPLATE_KWARGS_JSON" ]]; then
  GEN_ARGS+=(--chat-template-kwargs-json "$CHAT_TEMPLATE_KWARGS_JSON")
fi

python "$PROJECT_ROOT/scoring/v01/p_vllm_generation.py" "${GEN_ARGS[@]}"

EXTRACT_ARGS=(--model-name "$MODEL_NAME")
if [[ -n "$SYSTEMS" ]]; then
  EXTRACT_ARGS+=(--systems "$SYSTEMS")
fi

python "$PROJECT_ROOT/scoring/extract_model_outputs.py" "${EXTRACT_ARGS[@]}"

echo "Generation complete for $MODEL_NAME"
