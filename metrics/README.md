# `metrics/` (pipeline runtime outputs)

This directory holds **machine-written metric outputs of the evaluation pipeline**, not
analysis code:

- `evaluation_results.csv`: similarity metrics (CodeBLEU / ROUGE) written by
  `scoring/p_sim_score.py`.
- `all_metrics_combined.csv`: merged metrics written by `scoring/merge_metrics.py`.

It is distinct from the sibling **`analysis/`**, which is the analysis-and-figures workspace
(ranking/correlation scripts, plots, and their outputs). See `analysis/readme.md`.

> Formerly named `statistic/` (which was easy to confuse with the plural `statistics/`, now
> `analysis/`). Scripts that read/write here were updated accordingly:
> `scoring/p_sim_score.py`, `scoring/merge_metrics.py`, `scoring/rank_llm.py`,
> `scoring/evaluatePy.py`.
