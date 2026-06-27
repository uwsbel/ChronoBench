# Cutting a new contract version

A *contract* is the immutable spec that makes scores comparable: the tasks (`demo_data`, pinned by
content hash) + the grounding (`api.txt`) + the rubric (the 3 J-LLM prompts) + the judge config
(model + sampling). **Never edit an existing contract** (it would silently break the comparability
of everything scored under it). Instead, create a new versioned folder.

## Steps

1. **Pick a version id**, e.g. `v1.1-2027-newjudge` (a short, dated, reason-tagged name).
2. **Create `contracts/<version>/`** with:
   - `api.txt` - the API grounding snapshot for this version (copy `api/api.txt`, or the updated one).
   - `rubric/` - the three rubric prompts (`doc.txt`, `ref.txt`, `ref_doc.txt`); copy
     `chronobench/rubric/`, or the edited versions.
   - `contract.json` - `judge` {model, temperature, top_p, max_tokens}, the `api_doc`/`rubric_dir`
     names, and a `tasks` block (source=`demo_data`, n_systems/n_turns/n_tasks, and `sha256`).
     Use `contracts/v1.0-ieee-access-2026/contract.json` as the template.
3. **Pin the tasks hash** into `contract.json`:
   ```bash
   python -c "from chronobench.contract import hash_tasks; print(hash_tasks('demo_data'))"
   ```
   (non-task metadata like `manifest.json` and `README.md` are excluded automatically).
4. **Register it** in `CONTRACTS.md` (add a row) and state explicitly whether results under it are
   comparable to the prior version. They are NOT if you changed the rubric, `api.txt`, the judge
   model/sampling, or the `demo_data` content.
5. **Verify**:
   ```bash
   python -c "from chronobench.contract import load_contract; print(load_contract('<version>').verify_tasks())"
   python -m chronobench.score <model> --contract <version> --dry-run
   ```

## Re-baselining
Scores are not transferable across contract versions. To compare previously-scored agents under a
new contract, you must **re-run** them under it (`python -m chronobench.score <model> --contract <version>`).
