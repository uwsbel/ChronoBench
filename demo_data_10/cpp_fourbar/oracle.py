"""Independent oracle for the cpp_fourbar task (stdlib math only, NO Chrono).

Planar four-bar loop closure. Geometry from demo_MBS_fourbar.cpp (projectchrono/chrono
src/demos/mbs): crank pivot A0 = (0, 0), rocker pivot D0 = (8, -8), rocker length L3 = 8, and
the crank pin starts at (L1, 0) with the coupler running along +x to the rocker pin at (8, 0),
so the coupler length is L2 = 8 - L1 and the start is EXACTLY the extended (crank-coupler
collinear) configuration: the rocker begins at one of its swing extremes, psi0 = 90 deg.

For a crank angle phi, the pin P1 = L1 (cos phi, sin phi); the rocker pin P2 is the
intersection of circles (P1, L2) and (D0, L3), followed with branch continuity from the start;
psi = atan2(P2 - D0) is the rocker absolute angle and alpha = psi - psi0 its rotation. Swing
extremes occur at the collinear configurations, i.e. where |P2 - A0| = L1 + L2 = 8 (extended;
this is the start, alpha = 0) and |P2 - A0| = L2 - L1 (folded). With the crank motor at
constant speed the rocker oscillates once per crank revolution: period 2*pi/|omega|.

  turn 1 (translate): L1 = 2, omega = pi  -> alpha in [0, 0.5494] rad, period 2.0 s.
  turn 2 (modify):    L1 = 1, omega = pi  -> alpha in [0, 0.2534] rad (shorter crank, smaller
                      swing); an unmodified candidate fails the swing-amplitude band.
  turn 3 (extend):    L1 = 1, omega = 2pi -> same swing, period 1.0 s, plus the coupler
                      midpoint's vertical excursion as a new logged observable.

Grashof check (crank must fully rotate): s + l < p + q with s = L1, l = |A0 D0| = 11.3137.
L1 = 2: 13.31 < 14 OK; L1 = 1: 12.31 < 15 OK. (L1 = 3 would give 14.31 > 13: a non-Grashof
double-rocker that would fight the motor; rejected during design.)

Both extremes are also verified against a dense continuity-tracked sweep, which additionally
yields the coupler-midpoint excursion. Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/cpp_fourbar/oracle.py
"""
import json
import math

A0 = (0.0, 0.0)
D0 = (8.0, -8.0)
L3 = 8.0
PSI0 = math.pi / 2


def circle_intersections(c1, r1, c2, r2):
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h2 = r1 * r1 - a * a
    if h2 < 0:
        return []
    h = math.sqrt(max(0.0, h2))
    mx, my = c1[0] + a * dx / d, c1[1] + a * dy / d
    return [(mx + h * dy / d, my - h * dx / d), (mx - h * dy / d, my + h * dx / d)]


def sweep(L1, n=100000):
    """Track the rocker pin with branch continuity over one crank revolution."""
    L2 = 8.0 - L1
    p2 = (8.0, 0.0)
    alphas, cmy = [], []
    for k in range(n + 1):
        phi = 2 * math.pi * k / n
        p1 = (L1 * math.cos(phi), L1 * math.sin(phi))
        cands = circle_intersections(p1, L2, D0, L3)
        p2 = min(cands, key=lambda p: (p[0] - p2[0]) ** 2 + (p[1] - p2[1]) ** 2)
        alphas.append(math.atan2(p2[1] - D0[1], p2[0] - D0[0]) - PSI0)
        cmy.append(0.5 * (p1[1] + p2[1]))
    return min(alphas), max(alphas), max(cmy) - min(cmy)


def collinear_extremes(L1):
    """Closed-form swing extremes: rocker pin where crank + coupler are collinear."""
    out = []
    for d in (8.0, 8.0 - 2 * L1):        # extended (L1+L2 = 8 always) and folded (L2-L1)
        for p in circle_intersections(A0, d, D0, L3):
            out.append(math.atan2(p[1] - D0[1], p[0] - D0[0]) - PSI0)
    return out


out = {}
for name, L1, omega in (("turn1", 2.0, math.pi), ("turn2", 1.0, math.pi), ("turn3", 1.0, 2 * math.pi)):
    a_min, a_max, cmy_range = sweep(L1)
    ext = collinear_extremes(L1)
    # the sweep extremes must each match one collinear configuration
    assert min(abs(a_min - e) for e in ext) < 1e-3, (name, a_min, ext)
    assert min(abs(a_max - e) for e in ext) < 1e-3, (name, a_max, ext)
    out[name] = {"L1": L1, "L2": 8.0 - L1, "omega": round(omega, 4),
                 "alpha_min": round(a_min, 4), "alpha_max": round(a_max, 4),
                 "rocker_period_s": round(2 * math.pi / omega, 4),
                 "coupler_mid_y_range": round(cmy_range, 4)}
print(json.dumps(out, indent=2))
