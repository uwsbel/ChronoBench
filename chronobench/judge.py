"""Rule-based J-LLM evaluator for PyChrono virtual experiment scripts.

This is the reusable form of the judge that was previously locked inside
``scoring/engine/p_JLLM_score.py`` (model hardcoded, rubric duplicated three times, work done at
import time). Here a single ``evaluate_script(...)`` call returns the score plus the judge's
rationale, the rubric lives once in ``chronobench/rubric/*.txt``, and the model/provider are
parameters.

ChronoBench's purpose is to *evaluate and diagnose* a Chrono agent's virtual experiment scripts, so
the natural use is in a loop: an agent generates a script, ``evaluate_script`` scores it and explains
the deductions, the agent revises.

Three rubric modes:
    - ``"ref_api"`` : compare against the expert reference AND the API documentation (strongest).
    - ``"ref"``     : compare against the expert reference only.
    - ``"api"``     : compare against the API documentation only (use when no reference exists).

Example
-------
>>> from chronobench.judge import evaluate_script
>>> ev = evaluate_script(candidate_code, reference=truth_code, api_doc=api_text)
>>> ev.score, ev.mode
(72, 'ref_api')
>>> print(ev.rationale)   # the judge's per-criterion deductions, free text

Provider-agnostic: pass any OpenAI-compatible ``client`` (e.g. an ``openai.OpenAI`` pointed at
NVIDIA NIM, Together, vLLM, etc.). If omitted, an ``openai.OpenAI`` is built from
``OPENAI_API_KEY``.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path

# Judge sampling defaults, unchanged from the published pipeline (low variance).
DEFAULT_MODEL = os.getenv("CHRONOBENCH_JUDGE_MODEL", "gpt-4o-mini")
DEFAULT_TEMPERATURE = 0.2
DEFAULT_TOP_P = 0.7
DEFAULT_MAX_TOKENS = 12000

_RUBRIC_DIR = Path(__file__).resolve().parent / "rubric"

# mode -> (template filename, required template fields)
MODES: dict[str, tuple[str, tuple[str, ...]]] = {
    "ref_api": ("ref_api.txt", ("code", "reference_code", "api_documentation")),
    "ref": ("ref.txt", ("code", "reference_code")),
    "api": ("api_info.txt", ("code", "api_documentation")),
}

_SCORE_RE = re.compile(r"\[\[(\d+)\]\]")


@dataclass
class Evaluation:
    """Result of one J-LLM evaluation.

    Attributes:
        score: integer 0-100 parsed from the judge's ``[[x]]`` tag, or ``None`` if the judge
            did not emit a parsable score (e.g. an API error or a malformed response).
        rationale: the judge's full free-text explanation of the deductions (the per-criterion
            breakdown lives here; the rubric does not ask for structured sub-scores).
        mode: which rubric mode was used ("ref_api" | "ref" | "api").
        model: the judge model name.
        prompt: the exact prompt sent to the judge (kept for auditing/repro).
        raw: the raw judge response (== rationale; kept for symmetry with the legacy JSON dumps).
    """

    score: int | None
    rationale: str
    mode: str
    model: str
    prompt: str
    raw: str


def parse_score(text: str | None) -> int | None:
    """Extract the integer score from a judge response of the form '... [[42]]'.

    Returns the first match, or None if no ``[[number]]`` tag is present. (The legacy
    ``extract_scores_from_txt`` raised on a miss; here we return None so a single bad response
    does not abort a batch.)
    """
    if not text:
        return None
    m = _SCORE_RE.search(text)
    return int(m.group(1)) if m else None


def select_mode(reference: str | None, api_doc: str | None) -> str:
    """Pick the richest applicable rubric mode given what context is available."""
    if reference and api_doc:
        return "ref_api"
    if reference:
        return "ref"
    if api_doc:
        return "api"
    raise ValueError("evaluate_script needs at least one of `reference` or `api_doc`.")


def build_prompt(
    mode: str,
    code: str,
    reference: str | None = None,
    api_doc: str | None = None,
    rubric_dir=None,
) -> str:
    """Render the rubric prompt for a mode. Raises if required context is missing/empty.

    rubric_dir overrides the package's default rubric (e.g. to use a contract's frozen rubric
    snapshot); when None, the package `chronobench/rubric/` is used.
    """
    if mode not in MODES:
        raise ValueError(f"Unknown mode {mode!r}; expected one of {sorted(MODES)}.")
    fname, required = MODES[mode]
    values = {"code": code, "reference_code": reference, "api_documentation": api_doc}
    for field in required:
        key = {"code": "code", "reference_code": "reference", "api_documentation": "api_doc"}[field]
        if not values[field]:
            raise ValueError(f"mode {mode!r} requires non-empty `{key}`.")
    base = Path(rubric_dir) if rubric_dir else _RUBRIC_DIR
    template = (base / fname).read_text(encoding="utf-8")
    # Fill all placeholders; unused ones (per mode) are simply absent from the template.
    return template.format(
        code=code,
        reference_code=reference or "",
        api_documentation=api_doc or "",
    )


def _default_client():
    from openai import OpenAI  # imported lazily so importing chronobench never requires openai

    return OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


def evaluate_script(
    candidate: str,
    reference: str | None = None,
    api_doc: str | None = None,
    *,
    mode: str | None = None,
    model: str = DEFAULT_MODEL,
    client=None,
    temperature: float = DEFAULT_TEMPERATURE,
    top_p: float = DEFAULT_TOP_P,
    max_tokens: int = DEFAULT_MAX_TOKENS,
    rubric_dir=None,
) -> Evaluation:
    """Evaluate one candidate virtual experiment script with the rule-based J-LLM.

    Args:
        candidate: the agent-generated PyChrono code to score.
        reference: expert ground-truth code (enables "ref"/"ref_api" modes).
        api_doc: API documentation text, e.g. the contents of ``api/api.txt`` (enables
            "api"/"ref_api" modes).
        mode: force a rubric mode; if None, the richest applicable mode is selected.
        model: judge model name (default from $CHRONOBENCH_JUDGE_MODEL or "gpt-4o-mini").
        client: an OpenAI-compatible client; if None, one is built from $OPENAI_API_KEY.
        temperature, top_p, max_tokens: sampling params (defaults match the published pipeline).

    Returns:
        An :class:`Evaluation`. On an API error the score is None and ``rationale`` holds the
        error string, so callers can filter rather than crash mid-batch.
    """
    if mode is None:
        mode = select_mode(reference, api_doc)
    prompt = build_prompt(mode, candidate, reference, api_doc, rubric_dir=rubric_dir)

    if client is None:
        client = _default_client()

    try:
        completion = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            temperature=temperature,
            top_p=top_p,
            max_tokens=max_tokens,
            stream=False,
        )
        raw = completion.choices[0].message.content
    except Exception as exc:  # surface, do not crash a batch
        raw = f"ERROR: {exc}"

    return Evaluation(
        score=parse_score(raw),
        rationale=raw,
        mode=mode,
        model=model,
        prompt=prompt,
        raw=raw,
    )
