"""The ROUGE helper (the rouge_score-backed replacement for evaluate.load('rouge')) returns the
four expected F-measure keys as floats in [0, 1]. Offline and deterministic."""
from p_sim_score import rouge_fmeasures


def test_rouge_fmeasures_keys_and_range():
    out = rouge_fmeasures("a = 1\nprint(a)\n", "a = 1\nprint(a)\n")
    assert set(out) == {"rouge1", "rouge2", "rougeL", "rougeLsum"}
    assert all(isinstance(v, float) and 0.0 <= v <= 1.0 for v in out.values())


def test_rouge_identical_text_scores_one():
    text = "import chrono\nx = chrono.ChSystemNSC()\n"
    out = rouge_fmeasures(text, text)
    assert out["rouge1"] == 1.0
