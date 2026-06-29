"""Make the repo importable for the offline test suite.

`chronobench/` is a package, but `scoring/*.py` are standalone scripts (not an installed
package), so we put both the repo root and `scoring/` on sys.path. This mirrors how the CLIs
add PROJECT_ROOT themselves, and avoids needing an editable install.
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for _p in (REPO_ROOT, REPO_ROOT / "scoring"):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)
