> **Note on roles:** `analysis/` (this folder) is the **analysis and figures workspace**
> (ranking scripts, correlation analysis, plots, and their CSV/figure outputs). It is distinct
> from the sibling `metrics/`, which holds the **pipeline's metric outputs**
> (`evaluation_results.csv`, `all_metrics_combined.csv`) written by `scoring/p_sim_score.py` and
> `scoring/merge_metrics.py`. (These two folders were formerly the confusingly-named
> `statistics/` and `statistic/`.) See `metrics/README.md`.

This folder contains essential data and scripts for evaluating and analyzing the performance of various LLMs across multiple metrics, along with tools for visualization and ranking.

### CSV Data:
- The CSV files `evaluation_results.csv`, `filtered_pass_data.csv`, `pass.csv`, and `LLM.csv` contain detailed evaluation scores from multiple metrics, such as pass rates, code similarity, and other performance indicators. These files aggregate data across different test cases and LLMs to facilitate a comprehensive analysis.

### Python Scripts:
- The scripts `rank1.py`, `rank2.py`, `rank3.py`, `rank4.py`, and `rank5.py` are designed to extract specific metrics from the CSV files and rank the LLMs based on their performance in various categories, such as accuracy, code generation quality, and pass rates.

- `analyze_csv.py` provides a more in-depth analysis, including the extraction of key metrics and generating summaries or visualizations of the evaluation results.

- `correlation.py` calculates the Spearman correlation between the different evaluation metrics, helping to identify relationships between various performance measures and their impact on overall model effectiveness.

### Usage:
These files and scripts offer help with evaluating the performance of LLMs and visualizing trends or correlations in their results. One should be able to easily compare models and assess which ones perform better across different tasks or conditions.
