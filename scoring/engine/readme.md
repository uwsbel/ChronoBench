# `scoring/engine/` (batch J-LLM scoring)

This directory holds `p_JLLM_score.py`: the parallel **batch** J-LLM scorer that runs the
rule-based judge over many (model x system x turn) at once and writes the per-system score files
plus `output_llms/combined_evaluation_scores.csv`. It imports the rubric/judge logic from the
top-level `chronobench/` package (single source of truth) and runs only under `__main__`.

- To score a **single** agent, prefer `python -m chronobench.score <model>` (see `ONBOARDING.md`).
- A full batch run can incur significant API costs quickly.

**Generation moved.** The former per-provider generation drivers (Claude/GPT/Google/Mistral/NIM/
vLLM/DeepInfra variants) were consolidated into one parametrized, provider-agnostic generator:
`python -m chronobench.generate <label> [--provider ...] [--base-url ...]` (see its `--help`). Use
it to produce a model's `{first,second,third}_response.txt` outputs, then score them.
