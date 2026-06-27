#!/usr/bin/env bash
# Fetch the FROZEN published run (output_llms/, output_conversion/) into a lean clone.
#
# These large dirs live in an external archive (Zenodo record or GitHub Release), not in git;
# see DATA.md. Point this at the archive via $SIMBENCH_DATA_URL (or edit BASE_URL below). It is
# idempotent: a directory that is already present is left alone.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Base URL that directly serves the tarballs, e.g. a Zenodo record files URL or a GitHub Release
# assets URL. Fill this in (or export SIMBENCH_DATA_URL) once the archive is published.
BASE_URL="${SIMBENCH_DATA_URL:-PUT_ARCHIVE_BASE_URL_HERE}"
ARCHIVES=("output_llms.tar.gz" "output_conversion.tar.gz")

if [[ "$BASE_URL" == PUT_* ]]; then
  echo "ERROR: set SIMBENCH_DATA_URL (or edit BASE_URL) to the published archive base URL." >&2
  echo "       See DATA.md for the DOI / Release link." >&2
  exit 1
fi

for a in "${ARCHIVES[@]}"; do
  dir="${a%.tar.gz}"
  if [[ -d "$ROOT/$dir" ]]; then
    echo "[skip] $dir/ already present"
    continue
  fi
  echo "[fetch] $a"
  curl -fL "$BASE_URL/$a" -o "$ROOT/$a"
  tar -xzf "$ROOT/$a" -C "$ROOT"
  rm -f "$ROOT/$a"
done
echo "Done. Verify against the SHA256SUMS recorded in DATA.md if you want."
