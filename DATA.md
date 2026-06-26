# Data hosting notes

SimBench commits a large amount of generated data directly to git:

| Path | Size | Tracked files | What it is |
|------|------|---------------|------------|
| `output_llms/` | ~756 MB | ~41,300 | Per-model generated DT code + per-system score files for 30+ S-LLMs. |
| `output_conversion/` | ~84 MB | ~1,690 | Alpaca-format multi-turn conversation JSONs. |
| `.git/` | ~2.3 GB | — | History bloat from the above (and previously-removed paper builds). |

The `demo_data/` benchmark itself is small (~3 MB) and should stay in git. The two generated-output
directories are what make the repo heavy to clone.

## Recommended migration (NOT done automatically; needs Dan's go-ahead)

The goal is to keep the repo lean while preserving the data as a citable artifact:

1. **Publish the data externally.** Tar each directory and upload to a versioned, citable host:
   - Zenodo (gets a DOI; ideal to reference from the paper/README), or
   - a GitHub Release asset on `uwsbel/SimBench` (simpler, no DOI).
   ```bash
   tar -czf output_llms.tar.gz output_llms
   tar -czf output_conversion.tar.gz output_conversion
   ```
2. **Stop tracking them going forward** (removes from HEAD, keeps working-tree files):
   ```bash
   git rm -r --cached output_llms output_conversion
   # then uncomment the matching lines in .gitignore (see "Large generated data" section)
   ```
3. **Add a fetch path** so users can pull the data back (a `scripts/fetch_data.sh` that curls the
   Zenodo/Release tarballs and extracts them), and document it here + in `ONBOARDING.md`.
4. **Shrink history (optional, heavy, gated).** Removing the blobs from the existing 2.3 GB
   history needs `git filter-repo` (or BFG) followed by a **force-push**. On a public repo this
   rewrites shared history and invalidates existing clones/forks, so it must be coordinated and
   is **not** to be run without explicit confirmation.

Until step 1 is done, the data stays tracked as-is so nothing is lost. The `.gitignore` entries
for these directories are present but **commented out** so current tracking is unaffected;
uncomment them only after the data is hosted and untracked (step 2).
