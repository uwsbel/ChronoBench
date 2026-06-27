"""Canonical ChronoBench taxonomy: the 34 physical systems and their 5 categories.

This is the single source of truth for the system->category mapping. It mirrors the lists
historically hardcoded in ``scoring/evaluatePy.py`` (lines ~99-103); other code and the
``demo_data`` manifest should import from here rather than re-declaring the lists.

Each system contributes 3 turns (Turn 1 create, Turns 2-3 modify/extend), so there are
34 systems x 3 turns = 102 turn-level benchmark tasks.
"""

from __future__ import annotations

# category key -> (human-readable name, [systems])
CATEGORIES: dict[str, dict] = {
    "MBS": {
        "name": "Multibody dynamics (MBD)",
        "systems": ["pendulum", "slider_crank", "gear", "mass_spring_damper", "particles"],
    },
    "FEA": {
        "name": "Finite element analysis",
        "systems": ["beam", "buckling", "rotor", "tablecloth", "cable"],
    },
    "SEN": {
        "name": "Sensor integration",
        "systems": ["gps_imu", "lidar", "veh_app", "camera"],
    },
    "RBT": {
        "name": "Robotics dynamics",
        "systems": ["turtlebot", "viper", "curiosity", "vehros", "sensros", "handler"],
    },
    "VEH": {
        "name": "Vehicle dynamics",
        "systems": [
            "citybus", "feda", "gator", "hmmwv", "kraz", "art", "rigid_highway",
            "rigid_multipatches", "scm", "scm_hill", "uazbus", "m113", "sedan", "man",
        ],
    },
}

# flat system -> category map
_SYSTEM_TO_CATEGORY: dict[str, str] = {
    system: cat for cat, meta in CATEGORIES.items() for system in meta["systems"]
}

# sorted list of all 34 system names
SYSTEMS: list[str] = sorted(_SYSTEM_TO_CATEGORY)

TURNS = (1, 2, 3)


def category_of(system: str) -> str:
    """Return the category key (e.g. 'FEA') for a system name; raise if unknown."""
    try:
        return _SYSTEM_TO_CATEGORY[system]
    except KeyError as exc:
        raise KeyError(f"Unknown ChronoBench system: {system!r}") from exc


def all_systems() -> list[str]:
    """Return the 34 canonical system names, sorted."""
    return list(SYSTEMS)


if __name__ == "__main__":  # quick self-check
    n = len(SYSTEMS)
    assert n == 34, f"expected 34 systems, found {n}"
    print(f"{n} systems across {len(CATEGORIES)} categories ({n * 3} turn-level tasks)")
    for cat, meta in CATEGORIES.items():
        print(f"  {cat:4s} {len(meta['systems']):2d}  {meta['name']}")
