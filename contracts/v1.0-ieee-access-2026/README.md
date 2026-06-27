# Contract v1.0-ieee-access-2026 (frozen)

This is the benchmark **contract** that the published results were produced under. A "contract"
is the exact, immutable specification that makes scores comparable:

1. **Tasks**: the 34 systems x 3 turns in the top-level `demo_data/`, pinned by content hash
   (`tasks.sha256` in `contract.json`).
2. **Grounding**: `api.txt` (a frozen snapshot of the API documentation given to the judge).
3. **Rubric**: `rubric/{doc,ref,ref_doc}.txt` (frozen snapshots of the three J-LLM prompts).
4. **Judge config**: model `gpt-4o-mini`, `temperature=0.2`, `top_p=0.7`, `max_tokens=12000`.

Do not edit anything in this folder. Any change to the tasks, grounding, rubric, or judge config
breaks comparability with the published numbers and must be made as a NEW contract version (a new
`contracts/<version>/` folder), not an edit here. See the top-level `CONTRACTS.md`.

Reproduce the published configuration:

```bash
python -m chronobench.score <model> --contract v1.0-ieee-access-2026
```
