"""The score-merging step reads the current "Score API" headers (chronobench.score output) into
the internal score columns. Drives the real merge_metrics.main()."""
import importlib

_METRICS = "model,system,round,codebleu\nm1,pendulum,round_1,0.5\n"
_SCORES = ("Test Model,System,Round,Score API,Score Reference,Score Reference API\n"
           "m1,pendulum,first,80,70,90\n")
_INTERNAL = ["score_document", "score_reference", "score_reference_document"]


def _merge(tmp_path, scores_csv):
    mm = importlib.import_module("merge_metrics")
    (tmp_path / "metrics.csv").write_text(_METRICS, encoding="utf-8")
    (tmp_path / "scores.csv").write_text(scores_csv, encoding="utf-8")
    mm.EVAL_RESULTS = tmp_path / "metrics.csv"
    mm.COMBINED_SCORES = tmp_path / "scores.csv"
    mm.OUTPUT_FILE = tmp_path / "merged.csv"
    return mm.main()


def test_api_headers_merge_into_internal_columns(tmp_path):
    merged = _merge(tmp_path, _SCORES)
    assert all(c in merged.columns for c in _INTERNAL)
    row = merged.iloc[0]
    assert (row["score_document"], row["score_reference"], row["score_reference_document"]) == (80, 70, 90)
