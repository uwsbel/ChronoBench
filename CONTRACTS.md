# Benchmark contracts

A **contract** is the immutable specification a result is measured against: the tasks
(`demo_data`, pinned by content hash), the grounding (`api.txt`), the rubric (the three J-LLM
prompts), and the judge config (model + sampling). A score is comparable to another score only if
both were produced under the **same contract version**.

This is the living/frozen seam: the evaluator code (the `chronobench` package) and the pipeline
evolve freely; the contract is versioned. Evolving the rubric / api / judge / tasks does not edit
a contract, it creates a new one. To re-state old agents on a new contract you must re-run them.

| Version | Status | Judge | Tasks | Notes |
|---------|--------|-------|-------|-------|
| `v1.0-ieee-access-2026` | frozen (published) | gpt-4o-mini, T=0.2, top_p=0.7 | 34 systems x 3 turns | The IEEE Access 2026 baseline. `contracts/v1.0-ieee-access-2026/`. |

Usage:

```bash
# evaluate an agent under a specific contract (default is the published v1.0)
python -m chronobench.score <model> --contract v1.0-ieee-access-2026
```

When you create a new version (e.g. updated `api.txt`, more systems, a newer judge), add a
`contracts/<version>/` folder with its own `contract.json` + snapshots, add a row here, and state
explicitly whether/why it is or is not comparable to the prior version.
