"""Independent oracle for the fsi_object_drop task (stdlib math only, NO Chrono).

Archimedes' principle, a genuinely tight closed form for an SPH free-surface problem. A rigid
sphere of radius R and density rho_s dropped into water (rho_w = 1000) settles so the displaced
water weighs as much as the sphere. With draft h (submerged depth of the lowest point), the
submerged spherical-cap volume is
    V_cap(h) = pi h^2 (3R - h) / 3,
and flotation requires V_cap(h) = (rho_s/rho_w) V_sphere = (rho_s/rho_w)(4/3) pi R^3,
i.e. the cubic  x^3 - 3x^2 + 4*ratio = 0  in x = h/R. The settled CENTER sits at
    z_center - z_surface = R - h.

  turn 1 (create):  rho_s = 500  (ratio 0.5) -> x = 1 exactly: the center AT the free surface
                    (z_center - z_surface = 0). A pleasing special case: half-submerged.
  turn 2 (modify):  rho_s = 900  (ratio 0.9) -> x = 1.6084: center 0.6084 R = 0.0730 m BELOW
                    the surface. An unmodified candidate (center at 0) fails.
  turn 3 (extend):  rho_s = 2500 (ratio 2.5 > 1): no flotation solution exists; the sphere SINKS
                    and comes to rest on the tank floor: z_center = R above the floor. A
                    floating candidate fails high.

Cross-turn law: the settled center is strictly monotone in density. Calibration on the pinned
HIP build found a systematic SPH artifact worth naming: the BCE marker skin makes the sphere
hydrodynamically LARGER than its geometric radius (~half an initial spacing), adding ~35%
effective buoyant volume, so every floating case settles ~0.04-0.05 m HIGH of Archimedes at
spacing 0.025 (and a 1.2 density ratio still floats; hence turn 3 uses 2.5, which sinks
unambiguously). The graded bands are therefore anchored to this oracle but calibrated-and-frozen
around the measured truth values; see contract.json and CONTRACT.md.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/fsi_object_drop/oracle.py
"""
import json

R = 0.12
RHO_W = 1000.0


def draft_ratio(ratio):
    """Solve x^3 - 3x^2 + 4*ratio = 0 for x = h/R in [0, 2] (bisection; f(0)>0, f(2)<0... )."""
    if ratio >= 1.0:
        return None                      # denser than water: sinks
    lo, hi = 0.0, 2.0
    f = lambda x: x ** 3 - 3 * x ** 2 + 4.0 * ratio
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


out = {"R": R, "rho_water": RHO_W}
for name, rho in (("turn1", 500.0), ("turn2", 900.0), ("turn3", 2500.0)):
    x = draft_ratio(rho / RHO_W)
    if x is None:
        out[name] = {"rho_s": rho, "floats": False,
                     "settled_center": "tank floor + R (rests on the bottom)"}
    else:
        out[name] = {"rho_s": rho, "floats": True, "h_over_R": round(x, 4),
                     "center_rel_surface_m": round(R * (1.0 - x), 4)}
print(json.dumps(out, indent=2))
