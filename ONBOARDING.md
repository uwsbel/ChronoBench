# SimBench onboarding

SimBench **evaluates and diagnoses** Chrono agents: LLMs that generate **virtual experiment
scripts** (runnable PyChrono code that sets up and runs a simulation). It is not a training
framework. The core asset is a rule-based **LLM-as-a-judge (J-LLM)** that scores a generated
script against an expert reference and/or the API documentation, plus a frozen benchmark of 34
systems x 3 turns (102 tasks).

For the authoritative method, see the paper in [.claude/docs/](.claude/docs/) (and `CLAUDE.md`).
This file is the practical "how do I run it" guide.

## 1. Setup

```bash
conda env create -f environment.yml      # creates the `simbench` env
conda activate simbench
export OPENAI_API_KEY=sk-...              # judge defaults to gpt-4o-mini
```

- The judge model is configurable: `export SIMBENCH_JUDGE_MODEL=gpt-4o` (any OpenAI-compatible
  model; pass a non-OpenAI provider via `--base-url`).
- PyChrono is **not** in the env. It is only needed for the optional compile/run checks
  (`scoring/evaluatePy.py`), not for J-LLM scoring.

## 2. What lives where

| Path | Role |
|------|------|
| `demo_data/` | The benchmark: 34 systems x 3 turns. `manifest.json` indexes system -> category -> turn -> files. |
| `api/api.txt` | ~4k-token PyChrono API reference used as judge context. |
| `simbench/` | **Reusable evaluator** (this is the product): `judge.py` (`evaluate_dt`), `score.py` (CLI), `systems.py` (taxonomy). |
| `scoring/engine/` | The operational engine: S-LLM generation drivers + `p_JLLM_score.py` (batch judging). Not legacy. |
| `scoring/` (root) | The paper's analysis/plotting scripts + `rank_llm.py`. |
| `output_llms/` | Generated virtual experiment scripts + scores for 30+ published models (large; see `DATA.md`). |
| `metrics/` | Pipeline metric outputs (`evaluation_results.csv`, `all_metrics_combined.csv`). |
| `analysis/` | Analysis + figures workspace (ranking, correlation, plots). |

## 3. The evaluation pipeline

```
generate -> extract -> (compile) -> similarity -> J-LLM score -> rank
```

| Stage | Script | In -> Out |
|-------|--------|-----------|
| 1. Generate (optional) | `scoring/engine/*_generate_simulation.py` | prompts -> `output_llms/<model>/<system>/{first,second,third}_response.txt` |
| 2. Extract code | `scoring/extractPy.py` | `*_response.txt` -> `*_response.py` (+ cleaned) |
| 3. Compile/run (optional, needs PyChrono) | `scoring/evaluatePy.py` | `*_response.py` -> execution log (Compile@1) |
| 4. Similarity metrics | `scoring/p_sim_score.py` | cleaned code vs `cleaned_truth*.py` -> `metrics/evaluation_results.csv` |
| 5. J-LLM scoring | **`python -m simbench.score <model>`** (or legacy `scoring/engine/p_JLLM_score.py`) | `*_response.py` + `truth*.py` + `api.txt` -> per-system CSV + `output_llms/combined_evaluation_scores.csv` |
| 6. Rank | `scoring/rank_llm.py` | merges (4)+(5) -> rankings under `scoring/out/` |

## 4. Start here (pick your goal)

### (a) Reproduce the published evaluation
The generated outputs for 30+ models are already in `output_llms/`. To regenerate scores:
```bash
python scoring/p_sim_score.py                      # similarity metrics (edit its test_model_list)
python -m simbench.score claude-4-sonnet-20250514  # J-LLM scores for one model
python scoring/rank_llm.py                          # rankings
```

### (b) Evaluate a NEW agent against SimBench
1. Put your agent's generated code at:
   `output_llms/<your-agent>/<system>/{first,second,third}_response.py`
   for each of the 34 systems (see names in `demo_data/manifest.json`). If you only have raw
   `.txt` model responses, drop them as `*_response.txt` and run `scoring/extractPy.py` first.
2. Sanity-check the layout without spending API calls:
   ```bash
   python -m simbench.score <your-agent> --dry-run
   ```
3. Score it (writes a CSV in the standard schema, plus a by-category / by-turn summary):
   ```bash
   python -m simbench.score <your-agent>
   ```
4. Optional: `python scoring/rank_llm.py` to rank your agent against the published models.

### (c) Use the J-LLM as a standalone diagnostic (in an agent loop)
```python
from simbench.judge import evaluate_dt

ev = evaluate_dt(candidate_code, reference=truth_code, api_doc=open("api/api.txt").read())
print(ev.score)       # 0-100
print(ev.rationale)   # the judge's per-criterion deductions (free text) -> feed back to the agent
```
`reference` and/or `api_doc` are optional; the richest applicable rubric mode
(`ref_doc` > `ref` > `doc`) is chosen automatically.

## 5. Known rough edges

1. `scoring/p_sim_score.py` has a hardcoded `test_model_list` near the bottom; edit it before
   running. (`scoring/extractPy.py` and `python -m simbench.score` take the model on the
   command line, so prefer those.)
2. `output_llms/` and `output_conversion/` are large; see `DATA.md` for hosting guidance.
