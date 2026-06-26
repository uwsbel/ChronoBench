# `statistic/` (pipeline runtime outputs)

This directory holds **machine-written outputs of the evaluation pipeline**, not analysis code:

- `evaluation_results.csv` — similarity metrics (CodeBLEU / ROUGE) written by
  `scoring/p_sim_score.py`.
- `all_metrics_combined.csv` — merged metrics used downstream.
- `analysis_output/` — rankings written by `scoring/rank_llm.py` (when present).

It is intentionally separate from the sibling **`statistics/`** (plural), which is the
analysis and figures workspace (ranking/correlation scripts and plots). The near-identical
names are a historical wart; the two play different roles and are both kept. See
`statistics/readme.md`.

> The names are not renamed because `statistic/` is referenced by path in several scripts
> (`scoring/p_sim_score.py`, `scoring/rank_llm.py`, `scoring/v01/p_JLLM_score.py`); renaming
> would require coordinated edits across all of them.
