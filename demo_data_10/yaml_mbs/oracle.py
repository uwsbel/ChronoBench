"""Independent oracle for the yaml_mbs task (stdlib math only, NO Chrono).

The task asks for a slider-crank DECLARED IN CHRONO'S MBS-YAML SCHEMA (the candidate script writes
the model/simulation/solver YAML files, then loads them with chrono::parsers::ChParserMbsYAML and
runs). The physics is the same inline slider-crank identity used by the `slider_crank` task, with
different numbers, so the targets are exact closed forms of the declared parameters:

  crank pivot at the origin (revolute about +Y, motor-driven at constant omega), crank pin at
  radius r, connecting rod of length l realized as a DISTANCE constraint to the slider point,
  slider on a prismatic guide along +X:
      x(theta) = r cos(theta) + sqrt(l^2 - r^2 sin^2(theta))
      stroke   = 2 r                      (independent of l and omega)
      vmax     = max_theta |omega dx/dtheta|

Targets:
  turn 1 (create):  r = 0.25, l = 1.0, omega = pi     -> stroke 0.5
  turn 2 (modify):  r = 0.40, l = 1.0, omega = pi     -> stroke 0.8
  turn 3 (extend):  r = 0.40, l = 1.0, omega = 2*pi   -> stroke 0.8, peak slider speed

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/yaml_mbs/oracle.py
"""
import json
import math


def kinematics(r, l, omega, ngrid=200000):
    thetas = [2.0 * math.pi * i / ngrid for i in range(ngrid + 1)]
    xs = [r * math.cos(t) + math.sqrt(l ** 2 - (r * math.sin(t)) ** 2) for t in thetas]
    stroke = max(xs) - min(xs)

    def dxdt(t):
        s, c = math.sin(t), math.cos(t)
        return omega * (-r * s - (r * r * s * c) / math.sqrt(l ** 2 - (r * s) ** 2))

    vmax = max(abs(dxdt(t)) for t in thetas)
    return stroke, vmax


cases = {"turn1": (0.25, 1.0, math.pi), "turn2": (0.4, 1.0, math.pi), "turn3": (0.4, 1.0, 2.0 * math.pi)}
out = {}
for name, (r, l, om) in cases.items():
    stroke, vmax = kinematics(r, l, om)
    out[name] = {"r": r, "l": l, "omega": round(om, 6), "stroke_closed_form": 2.0 * r,
                 "stroke_numeric": round(stroke, 6), "peak_speed": round(vmax, 6)}
print(json.dumps(out, indent=2))
