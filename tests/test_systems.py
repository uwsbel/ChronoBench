"""The 34-system / 5-category taxonomy (102 turn-level tasks) is the benchmark's backbone."""
import pytest

from chronobench.systems import all_systems, category_of, CATEGORIES


def test_thirty_four_unique_systems():
    s = all_systems()
    assert len(s) == 34
    assert len(set(s)) == 34


def test_five_categories_summing_to_34():
    assert set(CATEGORIES) == {"MBS", "FEA", "SEN", "RBT", "VEH"}
    assert sum(len(meta["systems"]) for meta in CATEGORIES.values()) == 34


def test_category_of_known_systems():
    assert category_of("pendulum") == "MBS"
    assert category_of("beam") == "FEA"
    assert category_of("lidar") == "SEN"
    assert category_of("curiosity") == "RBT"
    assert category_of("hmmwv") == "VEH"


def test_category_of_unknown_raises():
    with pytest.raises(KeyError):
        category_of("not_a_real_system")
