"""Independent oracle for the plate_sinkage_scm task (stdlib math only, NO Chrono).

Bevameter / plate-sinkage on SCM (Bekker-Wong) deformable terrain, the first REALISTIC (terramechanics)
task, sourced from the SBEL reproducibility line of work. Independent physics: the Bekker-Wong
pressure-sinkage relation p = (Kc/b + Kphi) * z^n gives, for a flat plate of contact area A under vertical
load F (pressure p = F/A, characteristic width b = min plate dimension), the sinkage
    z = (p / (Kc/b + Kphi))^(1/n).

IMPORTANT (honest oracle): Chrono's SCM adds elastic + dynamic + discretization effects, so the ABSOLUTE
sinkage is measured to sit ~1.0-1.5x this ideal Bekker value (super-linear, slope ~1.15), NOT tightly
equal. So this task does NOT assert a tight number. It grades with (a) a COARSE band [0.5x, 2.5x] around
the Bekker value (independent order-of-magnitude), (b) the MONOTONIC settling of the plate, and (c) the
"softer soil / heavier load -> deeper" law encoded ACROSS turns via the shifting band. Parameter-first:
z is a pure function of the declared params (soil + plate + load), so eval-time randomization is free.

Run offline once; kept in-repo. conda run -n chronobench python demo_data_10/plate_sinkage_scm/oracle.py
"""
import json


def bekker_sinkage(F, area, width, Kphi, Kc=0.0, n=1.0):
    p = F / area
    return (p / (Kc / width + Kphi)) ** (1.0 / n)


# task params (MUST match contract.json `params` and the truth scripts) -- single declared source
bx = by = 0.2
area = bx * by
width = min(bx, by)
Kc, n = 0.0, 1.0
turns = {
    "turn1_baseline":    {"F": 500.0,  "Kphi": 2.0e6},
    "turn2_softer_soil": {"F": 500.0,  "Kphi": 5.0e5},   # 4x softer -> ~4x deeper
    "turn3_heavier":     {"F": 2000.0, "Kphi": 2.0e6},   # 4x load -> deeper
}
out = {}
for name, pp in turns.items():
    z = bekker_sinkage(pp["F"], area, width, pp["Kphi"], Kc=Kc, n=n)
    out[name] = {**pp, "bekker_sinkage_m": round(z, 6),
                 "coarse_band_m": [round(0.5 * z, 6), round(2.5 * z, 6)]}
print(json.dumps(out, indent=2))
