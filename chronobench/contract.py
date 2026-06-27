"""Benchmark contracts: the versioned, immutable spec a result is measured against.

A *contract* pins everything that makes scores comparable: the tasks (`demo_data`, by content
hash), the grounding (`api.txt`), the rubric (the three J-LLM prompts), and the judge config
(model + sampling). Contracts live in `contracts/<version>/`. The evaluator code evolves freely;
the contract is versioned. Changing any pinned element is a NEW contract version, not an edit.
"""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = _PKG_DIR.parent
CONTRACTS_DIR = PROJECT_ROOT / "contracts"
DEFAULT_CONTRACT = "v1.0-ieee-access-2026"


@dataclass
class Contract:
    version: str
    path: Path
    judge_model: str
    temperature: float
    top_p: float
    max_tokens: int
    api_doc_path: Path
    rubric_dir: Path
    tasks_source: str
    tasks_sha256: str
    raw: dict

    def read_api_doc(self) -> str:
        return self.api_doc_path.read_text(encoding="utf-8")

    def tasks_dir(self) -> Path:
        return PROJECT_ROOT / self.tasks_source

    def verify_tasks(self) -> bool:
        """True if the on-disk tasks still match the contract's pinned hash."""
        return hash_tasks(self.tasks_dir()) == self.tasks_sha256


def list_contracts() -> list[str]:
    if not CONTRACTS_DIR.is_dir():
        return []
    return sorted(p.name for p in CONTRACTS_DIR.iterdir() if (p / "contract.json").is_file())


def load_contract(version: str = DEFAULT_CONTRACT) -> Contract:
    """Load a contract by version name (looked up under contracts/) or by direct path."""
    base = Path(version)
    cdir = base if (base / "contract.json").is_file() else CONTRACTS_DIR / version
    cfg_path = cdir / "contract.json"
    if not cfg_path.is_file():
        avail = ", ".join(list_contracts()) or "(none)"
        raise FileNotFoundError(f"No contract {version!r} at {cdir}. Available: {avail}")
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    j = cfg.get("judge", {})
    t = cfg.get("tasks", {})
    return Contract(
        version=cfg.get("version", version),
        path=cdir,
        judge_model=j.get("model", "gpt-4o-mini"),
        temperature=j.get("temperature", 0.2),
        top_p=j.get("top_p", 0.7),
        max_tokens=j.get("max_tokens", 12000),
        api_doc_path=cdir / cfg.get("api_doc", "api.txt"),
        rubric_dir=cdir / cfg.get("rubric_dir", "rubric"),
        tasks_source=t.get("source", "demo_data"),
        tasks_sha256=t.get("sha256", ""),
        raw=cfg,
    )


def hash_tasks(tasks_dir) -> str:
    """Deterministic content hash of a tasks directory (excludes the generated manifest.json).

    Must match the algorithm used to populate `tasks.sha256` in a contract.json: walk in sorted
    order, and fold `relpath \\0 sha256(content)` for each file into one sha256. Non-task metadata
    files (the generated manifest, the folder README) are excluded so docs do not perturb the hash.
    """
    tasks_dir = Path(tasks_dir)
    h = hashlib.sha256()
    skip = {"manifest.json", "README.md"}
    files = []
    for dirpath, dirnames, filenames in os.walk(tasks_dir):
        dirnames.sort()
        for fn in sorted(filenames):
            if fn in skip:
                continue
            rel = os.path.relpath(os.path.join(dirpath, fn), tasks_dir).replace("\\", "/")
            files.append(rel)
    for rel in sorted(files):
        h.update(rel.encode())
        h.update(b"\0")
        h.update(hashlib.sha256((tasks_dir / rel).read_bytes()).digest())
    return h.hexdigest()


if __name__ == "__main__":  # quick self-check / hash recompute helper
    for v in list_contracts():
        c = load_contract(v)
        ok = c.verify_tasks()
        print(f"{v}: judge={c.judge_model} tasks_ok={ok} (pinned {c.tasks_sha256[:12]}...)")
