# `paper/` (frozen IEEE Access 2026 artifact)

This folder is the **frozen reproducibility artifact** for the published paper (SimBench, IEEE
Access 2026, doi:10.1109/ACCESS.2026.3685519). It is not part of the living tool; do not edit it.
Everything here corresponds to the one published run under contract `v1.0-ieee-access-2026`
(judge `gpt-4o-mini`; see `contracts/`).

Contents:
1. The analysis scripts that produced the paper's figures and tables: `multivariate_analysis.py`,
   `failure_mode_analysis.py`, `multiturn_delta_analysis.py`, `benchmark_analysis.py`,
   `metrics_analysis.py`, the `plot_*` and `compare_jllm_*` scripts, etc. They read the frozen
   `paper/out/*.csv`.
2. `out/`: the generated figures, ranking CSVs, LaTeX tables, and the multivariate report.
3. `combined_evaluation_scores.csv`: the merged per-(model, system, turn) J-LLM scores behind the
   published rankings (a small browsable copy; the full per-model generations are archived, see
   `DATA.md`).
4. `paper_addition_dataset.tex`: the dataset-complexity section.

The **living** benchmark and evaluator are at the repo top level (`chronobench/`, `scoring/`,
`demo_data/`, `contracts/`). Evaluate new agents through those (see `ONBOARDING.md`), not here.

Reproducing the published rankings uses the living pipeline (writes a fresh `scoring/out/`, which
is regenerated and distinct from this frozen `paper/out/`):

```bash
python scoring/rank_llm.py   # needs the published run (output_llms/, metrics/); see DATA.md
```
