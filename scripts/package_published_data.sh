#!/usr/bin/env bash
# Package the FROZEN published run for upload to Zenodo or a GitHub Release.
#
# Run this from a clone that still has output_llms/ and output_conversion/ (the ~840 MB that we
# want to move out of git). It writes tarballs + a checksum file to dist/ (git-ignored). After
# uploading, record the archive base URL and the SHA256SUMS in DATA.md, then run the untrack
# steps documented there.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/dist"
mkdir -p "$OUT"

for dir in output_llms output_conversion; do
  if [[ ! -d "$ROOT/$dir" ]]; then
    echo "ERROR: $dir/ not found under $ROOT (run from a clone that still has the published run)." >&2
    exit 1
  fi
  echo "[tar] $dir -> dist/$dir.tar.gz"
  tar -czf "$OUT/$dir.tar.gz" -C "$ROOT" "$dir"
done

( cd "$OUT" && sha256sum ./*.tar.gz > SHA256SUMS && echo "[checksums]" && cat SHA256SUMS )
echo
echo "Next: upload dist/*.tar.gz to Zenodo (DOI) or a GitHub Release, then put the base URL and"
echo "the SHA256SUMS into DATA.md, and follow the untrack steps there."
