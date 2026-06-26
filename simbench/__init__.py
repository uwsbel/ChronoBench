"""SimBench: evaluation and diagnosis of LLM-generated PyChrono digital twins.

This package exposes the SimBench *evaluator* as a reusable library, distinct from the
research/analysis scripts under ``scoring/``. SimBench's purpose is to **evaluate and
diagnose** Chrono agents (LLMs that generate digital-twin code), not to train them.

Public API:
    - ``simbench.systems``  : the canonical 34 systems and 5 categories (single source of truth).
    - ``simbench.judge``    : the rule-based J-LLM evaluator, ``evaluate_dt(...)``.
    - ``simbench.score``    : a CLI to evaluate one agent's outputs against the benchmark.
"""

from .systems import CATEGORIES, SYSTEMS, category_of, all_systems  # noqa: F401

__all__ = ["CATEGORIES", "SYSTEMS", "category_of", "all_systems"]
__version__ = "0.1.0"
