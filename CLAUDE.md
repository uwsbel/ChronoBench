# SimBench

SimBench is a benchmark for evaluating and diagnosing how well simulator-oriented LLMs
(S-LLMs) generate **digital twins (DTs)**, i.e. simulator-ready scripts/configs, for
multi-physics simulation. Given a set of S-LLMs, it ranks them by DT quality using a
rule-based **LLM-as-a-judge (J-LLM)** that combines predefined rubrics with human-in-the-loop
calibration. It is demonstrated with the open-source **Chrono** multi-physics simulator, but
the methodology is simulator-agnostic.

## Source of truth (read these first)

The authoritative description of this project is the **published IEEE Access paper**, kept
in-repo:

1. `.claude/docs/2026Jingquan-SimBench.pdf` — the published PDF. **This is authoritative.**
2. `.claude/docs/tex-source-uber.tex` — the full paper LaTeX source, flattened into one file
   (all sections inlined). Best for reading and `grep` (methodology, equations, claims,
   exact numbers, the S-LLM/J-LLM prompt templates, appendices).

Before reasoning about the methodology, claims, numbers, terminology, or prompt design,
consult these rather than relying on memory or the README. Where the `.tex` source and the
PDF disagree, **trust the PDF** — the source still carries some commented-out editorial
notes, `\updatedText{}` revision macros, and minor typos that do not appear in the
rendered/published paper.

Note: `README.md` is a summary and can lag the paper; the paper governs.

## Where things live (code)

1. `api/api.txt` — condensed Chrono/PyChrono API reference fed to the J-LLM as context.
2. `demo_data/` — expert ground-truth DTs: 34 physical systems, each with three turns
   (`truth1.py`/`truth2.py`/`truth3.py`) of increasing complexity (102 tasks total).
3. `output_llms/` and `output_conversion/` — S-LLM generations and converted conversations.
4. `scoring/` — the evaluation pipeline (extract → compile/run → similarity + J-LLM scoring →
   ranking) and the analysis/plotting scripts.
5. `statistics/` — statistical analysis (rankings, correlations) over the scored results.
6. `visualization/` — pipeline and demo-overview figures.

## Conventions

- Python here is managed with **conda** (prefer `conda`/`conda-forge` over `pip`).
