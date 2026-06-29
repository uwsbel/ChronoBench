"""The frozen v1.0 contract still verifies: pinned judge/config, the demo_data hash matches,
and the hash is deterministic. This is what guarantees a score is comparable to the paper."""
from chronobench.contract import load_contract, hash_tasks, list_contracts, DEFAULT_CONTRACT


def test_default_contract_loads_with_pinned_judge():
    c = load_contract(DEFAULT_CONTRACT)
    assert c.version == "v1.0-ieee-access-2026"
    assert c.judge_model == "gpt-4o-mini"
    assert (c.temperature, c.top_p, c.max_tokens) == (0.2, 0.7, 12000)


def test_pinned_task_counts():
    t = load_contract(DEFAULT_CONTRACT).raw["tasks"]
    assert (t["n_systems"], t["n_turns"], t["n_tasks"]) == (34, 3, 102)


def test_demo_data_still_matches_pinned_hash():
    assert load_contract(DEFAULT_CONTRACT).verify_tasks() is True


def test_hash_tasks_is_deterministic():
    c = load_contract(DEFAULT_CONTRACT)
    h1 = hash_tasks(c.tasks_dir())
    h2 = hash_tasks(c.tasks_dir())
    assert h1 == h2 == c.tasks_sha256


def test_v1_is_listed():
    assert DEFAULT_CONTRACT in list_contracts()
