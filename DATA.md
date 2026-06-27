# Data hosting notes

SimBench commits a large amount of generated data directly to git:

| Path | Size | Tracked files | What it is |
|------|------|---------------|------------|
| `output_llms/` | ~756 MB | ~41,300 | Per-model generated virtual experiment scripts + per-system score files for 30+ S-LLMs. |
| `output_conversion/` | ~84 MB | ~1,690 | Alpaca-format multi-turn conversation JSONs. |
| `.git/` | ~2.3 GB | n/a | History bloat from the above (and previously-removed paper builds). |

The `demo_data/` benchmark itself is small (~3 MB) and should stay in git. The two generated-output
directories are what make the repo heavy to clone.

## Archive location

- Record: https://zenodo.org/records/20974275
- DOI: `10.5281/zenodo.20974275` (https://doi.org/10.5281/zenodo.20974275)
- Archive base URL (Zenodo files): `https://zenodo.org/records/20974275/files`
- Checksums (SHA256):
  - `output_llms.tar.gz`: `29bc3c7f588d37c1ad839b92628d0ffcf7a30132944b56e4442fef4a31b7f591`
  - `output_conversion.tar.gz`: `7796e86dbae651570ab49e9598d6c8216dc34c72bf57c070eb72cf412222ba08`

## Recommended migration

The goal is to keep the repo lean while preserving the data as a citable artifact. The packaging
and fetch scripts are ready (`scripts/`); the upload and the untracking are gated on Dan.

1. **Package** (ready): writes tarballs + checksums to `dist/` (git-ignored).
   ```bash
   bash scripts/package_published_data.sh
   ```
2. **Publish** (needs Dan): upload `dist/output_llms.tar.gz` and `dist/output_conversion.tar.gz` to
   Zenodo (DOI; ideal for citing) or a GitHub Release on the repo. Record the base URL + DOI +
   checksums in the section above.
3. **Fetch path** (ready): once published, anyone restores the data with
   ```bash
   SIMBENCH_DATA_URL=<archive base url> bash scripts/fetch_published_data.sh
   ```
4. **Stop tracking** (needs Dan; do only after steps 2-3 work): removes from HEAD, keeps the
   working-tree files.
   ```bash
   git rm -r --cached output_llms output_conversion
   # then uncomment the output_llms/ and output_conversion/ lines in .gitignore
   ```
5. **Shrink history** (optional, heavy, separate explicit OK): removing the blobs from the ~2.3 GB
   history needs `git filter-repo` (or BFG) + a **force-push**. On a public repo this rewrites
   shared history and invalidates existing clones/forks, so it must be coordinated and is **not**
   run without explicit confirmation.

Until step 4, the data stays tracked as-is so nothing is lost. The `.gitignore` entries for these
directories are present but **commented out** so current tracking is unaffected; uncomment them
only after the data is hosted and untracked.
