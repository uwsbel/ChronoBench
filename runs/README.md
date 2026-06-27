# `runs/` (new-agent evaluation outputs)

Put outputs from evaluating a **new** agent here, so they never overwrite the frozen published
run in `output_llms/` / `metrics/`. The contents of this folder are git-ignored (this README is
the only tracked file).

Evaluate a new agent against the published contract:

```bash
# 1. place the agent's generated scripts at runs/<agent>/<system>/{first,second,third}_response.py
# 2. score them (uses the v1.0 contract by default, so results are comparable to the paper)
python -m chronobench.score <agent> --responses-dir runs --contract v1.0-ieee-access-2026
# 3. (optional) similarity metrics into a runs-local CSV, not the frozen metrics/
python scoring/p_sim_score.py <agent> --responses-dir runs --out runs/<agent>/sim_metrics.csv
```
