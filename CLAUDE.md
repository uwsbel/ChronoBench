# ChronoBench

ChronoBench (published as SimBench, IEEE Access 2026) is a benchmark for evaluating and
diagnosing how well simulator-oriented LLMs
(S-LLMs) generate **virtual experiment scripts** (simulator-ready scripts/configs that set up
and run a simulation) for multi-physics simulation. Given a set of S-LLMs, it ranks them by the
quality of those scripts using a rule-based **LLM-as-a-judge (J-LLM)** that combines predefined
rubrics with human-in-the-loop calibration. It is demonstrated with the open-source **Chrono**
multi-physics simulator, but the methodology is simulator-agnostic. (The published paper calls
these artifacts "digital twins"; in this repo we use the term "virtual experiment script".)

## Source of truth (read these first)

The authoritative description of this project is the **published IEEE Access paper**, kept
in-repo:

1. `.claude/docs/2026Jingquan-SimBench.pdf`: the published PDF. **This is authoritative.**
2. `.claude/docs/tex-source-uber.tex`: the full paper LaTeX source, flattened into one file
   (all sections inlined). Best for reading and `grep` (methodology, equations, claims,
   exact numbers, the S-LLM/J-LLM prompt templates, appendices).

Before reasoning about the methodology, claims, numbers, terminology, or prompt design,
consult these rather than relying on memory or the README. Where the `.tex` source and the
PDF disagree, **trust the PDF**: the source still carries some commented-out editorial
notes, `\updatedText{}` revision macros, and minor typos that do not appear in the
rendered/published paper.

Note: `README.md` is a summary and can lag the paper; the paper governs.

## Where things live (code)

1. `chronobench/` (package): the reusable evaluator, `judge.evaluate_script(...)`, the
   `python -m chronobench.score` CLI, and the system/category taxonomy (`systems.py`).
2. `api/api.txt`: condensed Chrono/PyChrono API reference fed to the J-LLM as context.
3. `demo_data/`: expert ground-truth scripts, 34 physical systems, each with three turns
   (`truth1.py`/`truth2.py`/`truth3.py`) of increasing complexity (102 tasks total).
4. `output_llms/` and `output_conversion/`: S-LLM generations and converted conversations.
5. `scoring/`: the living pipeline (`extractPy`, `evaluatePy`, `p_sim_score`, `clean_truth`,
   `merge_metrics`, `generate_manifest`) plus `rank_llm.py`; `scoring/engine/` holds the
   generation drivers and the batch J-LLM scorer (`p_JLLM_score.py`).
6. `contracts/`: versioned benchmark contracts (see `CONTRACTS.md`); `v1.0-ieee-access-2026` is
   the frozen published baseline that pins judge config + api/rubric snapshots + the `demo_data` hash.
7. `paper/`: the FROZEN IEEE Access 2026 artifact (the paper's figure/table analysis scripts +
   `out/` + a summary CSV). Reproducibility only, not the living tool.
8. `metrics/`: living pipeline metric outputs; `runs/`: new-agent eval outputs (git-ignored).
9. `visualization/`: pipeline and overview figures used by the README and the paper.

## Conventions

- Python here is managed with **conda** (prefer `conda`/`conda-forge` over `pip`).
