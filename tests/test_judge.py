"""J-LLM mode machinery: keys, selection, the doc->api rename + back-compat aliases,
score parsing, and rubric-template rendering incl. the frozen-contract filename fallback."""
import pytest

from chronobench.judge import MODES, select_mode, parse_score, build_prompt
from chronobench.contract import load_contract, DEFAULT_CONTRACT


def test_mode_keys_are_renamed():
    assert sorted(MODES) == ["api", "ref", "ref_api"]


def test_select_mode_picks_richest_available():
    assert select_mode("ref", "doc") == "ref_api"
    assert select_mode("ref", None) == "ref"
    assert select_mode(None, "doc") == "api"


def test_select_mode_requires_some_context():
    with pytest.raises(ValueError):
        select_mode(None, None)


def test_parse_score():
    assert parse_score("per-criterion deductions ... final score [[73]]") == 73
    assert parse_score("the judge forgot to emit a score") is None
    assert parse_score(None) is None


def test_build_prompt_renders_each_mode_against_package_rubric():
    assert "APIDOC" in build_prompt("api", "CODE", api_doc="APIDOC")
    assert "REF" in build_prompt("ref", "CODE", reference="REF")
    p = build_prompt("ref_api", "CODE", reference="REF", api_doc="APIDOC")
    assert "REF" in p and "APIDOC" in p


def test_build_prompt_raises_on_missing_context():
    with pytest.raises(ValueError):
        build_prompt("api", "CODE")          # no api_doc
    with pytest.raises(ValueError):
        build_prompt("ref", "CODE")          # no reference


def test_build_prompt_uses_contract_rubric():
    # The v1.0 contract's rubric files are named api_info.txt / ref.txt / ref_api.txt (matching the
    # package), so build_prompt loads them directly when handed the contract's rubric_dir.
    rd = str(load_contract(DEFAULT_CONTRACT).rubric_dir)
    assert "APIDOC" in build_prompt("api", "CODE", api_doc="APIDOC", rubric_dir=rd)
    p = build_prompt("ref_api", "CODE", reference="REF", api_doc="APIDOC", rubric_dir=rd)
    assert "REF" in p and "APIDOC" in p
