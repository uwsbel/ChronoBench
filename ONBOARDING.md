# ChronoBench onboarding

ChronoBench **evaluates and diagnoses** Chrono agents: LLMs that generate **virtual experiment
scripts** (runnable PyChrono code that sets up and runs a simulation). It is not a training
framework. The core asset is a rule-based **LLM-as-a-judge (J-LLM)** that scores a generated
script against an expert reference and/or the API documentation, plus a frozen benchmark of 34
systems x 3 turns (102 tasks).

For the authoritative method, see the paper in [.claude/docs/](.claude/docs/) (and `CLAUDE.md`).
This file is the practical "how do I run it" guide.

## 1. Setup

```bash
conda env create -f environment.yml      # creates the `chronobench` env
conda activate chronobench
export OPENAI_API_KEY=sk-...              # judge defaults to gpt-4o-mini
```

- The judge model is configurable: `export CHRONOBENCH_JUDGE_MODEL=gpt-4o` (any OpenAI-compatible
  model; pass a non-OpenAI provider via `--base-url`).
- PyChrono is **not** in the env. It is only needed for the optional compile/run checks
  (`scoring/evaluatePy.py`), not for J-LLM scoring.

## 2. What lives where

| Path | Role |
|------|------|
| `demo_data/` | The benchmark: 34 systems x 3 turns. `manifest.json` indexes system -> category -> turn -> files. |
| `api/api.txt` | ~4k-token PyChrono API reference used as judge context. |
| `chronobench/` | **Reusable evaluator** (this is the product): `judge.py` (`evaluate_script`), `score.py` (CLI), `systems.py` (taxonomy). |
| `scoring/engine/` | The operational engine: S-LLM generation drivers + `p_JLLM_score.py` (batch judging). Not legacy. |
| `scoring/` (root) | The living pipeline (extract/compile/sim-score/clean/merge) + `rank_llm.py`. |
| `contracts/` | Versioned benchmark contracts (see `CONTRACTS.md`); `v1.0-ieee-access-2026` is the frozen baseline. |
| `paper/` | FROZEN IEEE Access 2026 artifact: paper figure/table analysis + `out/` + summary CSV. Not the tool. |
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
| 5. J-LLM scoring | **`python -m chronobench.score <model>`** (or legacy `scoring/engine/p_JLLM_score.py`) | `*_response.py` + `truth*.py` + `api.txt` -> per-system CSV + `output_llms/combined_evaluation_scores.csv` |
| 6. Rank | `scoring/rank_llm.py` | merges (4)+(5) -> rankings under `scoring/out/` |

## 4. Start here (pick your goal)

### (a) Reproduce the published evaluation
The published run lives on Zenodo (`10.5281/zenodo.20974275`), not in git. Fetch it first, then
regenerate scores:
```bash
bash scripts/fetch_published_data.sh                # restores output_llms/ + output_conversion/
python scoring/p_sim_score.py                       # similarity metrics for all models present
python -m chronobench.score claude-4-sonnet-20250514   # J-LLM scores for one model (v1.0 contract)
python scoring/rank_llm.py                           # rankings
```

### (b) Evaluate a NEW agent against ChronoBench
New-agent outputs go under `runs/` so they never touch the frozen published run (see
`runs/README.md`). Scoring uses a **contract** (default `v1.0-ieee-access-2026`), so results are
comparable to the paper.
1. Put your agent's generated code at:
   `runs/<your-agent>/<system>/{first,second,third}_response.py`
   for each of the 34 systems (see names in `demo_data/manifest.json`). If you only have raw
   `.txt` model responses, drop them as `*_response.txt` and run
   `python scoring/extractPy.py <your-agent> --responses-dir runs` first.
2. Sanity-check the layout without spending API calls:
   ```bash
   python -m chronobench.score <your-agent> --responses-dir runs --dry-run
   ```
3. Score it (uses the v1.0 contract; writes a CSV in the standard schema + a by-category/turn summary):
   ```bash
   python -m chronobench.score <your-agent> --responses-dir runs
   ```
4. Contracts are listed in `CONTRACTS.md`; pass `--contract <version>` to score under a different one.

### (c) Use the J-LLM as a standalone diagnostic (in an agent loop)
```python
from chronobench.judge import evaluate_script

ev = evaluate_script(candidate_code, reference=truth_code, api_doc=open("api/api.txt").read())
print(ev.score)       # 0-100
print(ev.rationale)   # the judge's per-criterion deductions (free text) -> feed back to the agent
```
`reference` and/or `api_doc` are optional; the richest applicable rubric mode
(`ref_doc` > `ref` > `doc`) is chosen automatically.

## 5. Known rough edges

1. `scoring/p_sim_score.py` has a hardcoded `test_model_list` near the bottom; edit it before
   running. (`scoring/extractPy.py` and `python -m chronobench.score` take the model on the
   command line, so prefer those.)
2. `output_llms/` and `output_conversion/` are large; see `DATA.md` for hosting guidance.
