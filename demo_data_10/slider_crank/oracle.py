"""Independent oracle for the slider_crank task (stdlib math only, NO Chrono).

Inline slider-crank kinematics: with crank angle theta = omega*t, crank radius r, and rod length l, the
piston position along the slide axis (measured from the crank center) is
    x(theta) = r*cos(theta) + sqrt(l^2 - r^2*sin^2(theta)).
From this:
  - stroke (peak-to-peak piston travel) = 2*r exactly (independent of l and omega),
  - peak piston speed = max_theta |omega * dx/dtheta|,  dx/dtheta = -r*sin - r^2*sin*cos/sqrt(l^2 - r^2 sin^2).

Targets:
  turn 1 (create):  r=0.4, l=1.5, omega=pi    -> stroke 0.8
  turn 2 (modify):  r=0.6, l=1.5, omega=pi    -> stroke 1.2
  turn 3 (extend):  r=0.6, l=1.5, omega=2*pi  -> stroke 1.2, peak piston speed

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/slider_crank/oracle.py
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


cases = {"turn1": (0.4, 1.5, math.pi), "turn2": (0.6, 1.5, math.pi), "turn3": (0.6, 1.5, 2.0 * math.pi)}
out = {}
for name, (r, l, om) in cases.items():
    stroke, vmax = kinematics(r, l, om)
    out[name] = {"r": r, "l": l, "omega": round(om, 6), "stroke_closed_form": 2.0 * r,
                 "stroke_numeric": round(stroke, 6), "peak_speed": round(vmax, 6)}
print(json.dumps(out, indent=2))
