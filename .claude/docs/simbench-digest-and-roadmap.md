# SimBench: organizational digest + roadmap

Internal reference (kept in `.claude/docs/` alongside the paper). Two parts: (A) how the repo
is organized, and (B) analysis of how it can better serve its purpose, plus what was done.

**Purpose (authoritative):** SimBench exists to **evaluate and diagnose** Chrono agents (LLMs
that generate digital-twin PyChrono code). It is **not** a training framework. The reuse of its
data as preference/training material is a byproduct the paper mentions, not the mission. SimBench
serves "better Chrono agents" by *measuring and diagnosing* them.

---

## Part A: How the repo is organized

Four functional layers plus the data. The key non-obvious point: the **engine and the analysis
are separate**, and the directory names hide it.

1. **Benchmark data (`demo_data/`)** - 34 systems x 3 turns (102 tasks). Per system, 14 files:
   `input{1,2,3}.txt` (prompts), `truth{1,2,3}.py` (expert reference DTs), `cleaned_truth*.py`
   (for similarity metrics), `pyinput{2,3}.py` (code given to the agent on modify turns),
   `output{1,2,3}.json` (Alpaca conversations). Indexed by `demo_data/manifest.json`. Categories
   were hardcoded in `scoring/evaluatePy.py:94-103`; now centralized in `simbench/systems.py`.
2. **API context (`api/api.txt`)** - ~4k-token PyChrono reference (2-level LLM summarization),
   used as J-LLM grounding.
3. **Operational engine (`scoring/engine/`)** - despite the name, the live machinery: S-LLM
   generation drivers + `p_JLLM_score.py` (batch judging). Now imports `simbench/`.
4. **Analysis layer (`scoring/` root)** - ~30 scripts that built the paper's figures/tables
   (`multivariate_analysis.py`, `failure_mode_analysis.py`, `rank_llm.py`, `plot_*`).
5. **Generated corpus + results** - `output_llms/` (~756 MB, 30+ models),
   `output_conversion/` (~84 MB), `metrics/` (pipeline metric outputs), `analysis/`
   (analysis + figures workspace), `visualization/` (figures).

**Pipeline:** generate -> `extractPy.py` -> `evaluatePy.py` (compile/run) -> `p_sim_score.py`
(CodeBLEU/ROUGE) -> J-LLM scoring -> `rank_llm.py`. See `ONBOARDING.md` for exact commands.

---

## Part B: Serving the purpose better

### Strategic frame (from the paper)
The multivariate analysis is the compass: the **evaluation/feedback protocol dominates**
outcomes (multi-turn "round" ~31% of score variance; Turn 1->2 feedback +29 pts) while **model
choice explains only ~3-5%**. So the value for building Chrono agents lives in the **diagnostic
judge, the expert reference DTs, and the API grounding**, not in the leaderboard. The work is to
*liberate those assets into reusable form*, not to chase a better base model.

### Gaps that were addressed
1. The repo was a frozen paper artifact (demo_data unchanged since 2025-05-30); the assets were
   locked in research scripts.
2. The judge (the highest-value component) was a monolith: rubric duplicated 3x, `gpt-4o-mini`
   hardcoded, API key printed, work done at import time, not importable.
3. No reproducible env, no runbook, hygiene issues (no `.gitignore`; committed `.idea/`,
   `extraction.log` leaking a user path, `__pycache__`).

### What was implemented (eval-only scope)
- **P0 hygiene + onboarding:** root `.gitignore` + untracked cruft; `ONBOARDING.md` (pipeline +
  three evaluation start-paths: reproduce / evaluate a new agent / standalone diagnostic);
  `demo_data/manifest.json` (+ generator); conda `environment.yml` + completed pinned
  `requirements.txt` (added missing scipy/scikit-learn/seaborn/statsmodels/tiktoken).
- **P1 reusable evaluator (centerpiece):** new `simbench/` package -
  `judge.py::evaluate_dt(candidate, reference=None, api_doc=None, mode=..., model=..., client=...)`
  returns `{score, rationale, mode, model, prompt, raw}`; provider-agnostic; rubric factored into
  versioned `simbench/rubric/*.txt`. `simbench/systems.py` is the taxonomy single-source.
  `python -m simbench.score <model>` evaluates one agent's outputs (with `--dry-run`) and writes
  a CSV in the published schema (so `rank_llm.py` consumes it). `scoring/engine/p_JLLM_score.py`
  refactored to use the package, guarded under `__main__`, model via CLI/env, no key print.
- **P2 structure + data hosting:** renamed the misleading dirs (`scoring/v01` -> `scoring/engine`,
  `statistic` -> `metrics`, `statistics` -> `analysis`) and updated every reference; `DATA.md`
  describes the ~840 MB hosting migration.

### Remaining / gated
1. **Data hosting** (`DATA.md`): publish `output_llms`/`output_conversion` to Zenodo or a Release,
   untrack, then optionally rewrite history (force-push, **gated on explicit OK**).
2. **Longitudinal tracking** (paper future work): a cadence to score new models over time.
3. **Add-a-system guide** if the benchmark is ever extended (note: extending breaks comparability
   with published results).

### Deliberately out of scope (off-mission)
Training/SFT/preference curation of `output_llms/`, HuggingFace training loaders, and RAG
context packs - those build/train agents rather than evaluate them.
