"""Independent oracle for the mjcf_cartpole task (stdlib math only, NO Chrono, NO MuJoCo).

A pendulum hinged under a cart that is free to slide (the classic cart-pole, pole hanging DOWN
so it oscillates). Linearized Lagrangian dynamics about the hanging equilibrium: eliminating
the cart acceleration through horizontal momentum conservation gives

    omega^2 = m g d / (I_h - m^2 d^2 / (M + m)),    T = 2 pi / omega,

with m, d, I_h the pole mass, hinge-to-COM distance, and moment of inertia ABOUT THE HINGE
(I_h = I_com + m d^2), and M the cart mass. The mass-ratio term m^2 d^2/(M+m) is the cart
recoil: a FREE cart lets the pendulum swing faster than the same pendulum on a locked base
(finite-M correction), and a LIGHTER cart recoils more, faster still. Locking the cart
(M -> infinity) recovers the plain compound-pendulum omega^2 = m g d / I_h.

Model (authored MJCF, source/cartpole_v1.xml): pole = uniform rod, m = 0.5, length 1 hanging
from the hinge, d = 0.5, I_com = m L^2/12 = 0.0416667, I_h = m L^2/3 = 0.1666667; release from
0.15 rad. Horizontal momentum starts at zero, so the SYSTEM COM x stays constant while cart and
pole counter-oscillate (a logged conservation invariant on turns 1-2).

  turn 1 (convert): cart M = 2.0   -> T = 1.5101 s.
  turn 2 (modify):  cart M = 0.5   -> T = 1.2949 s (lighter cart, more recoil, faster).
  turn 3 (extend):  cart LOCKED    -> T = 1.6379 s (pure compound pendulum; also the
                    import_urdf free-swing value, a nice cross-task consistency check).

Amplitude correction at 0.15 rad is (1 + theta0^2/16) ~ 1.0014: far inside the bands.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/mjcf_cartpole/oracle.py
"""
import json
import math

G = 9.81
M_POLE = 0.5
D = 0.5
I_COM = M_POLE * 1.0 ** 2 / 12.0
I_H = I_COM + M_POLE * D * D


def period(cart_mass):
    if cart_mass is None:               # locked cart
        w2 = M_POLE * G * D / I_H
    else:
        w2 = M_POLE * G * D / (I_H - (M_POLE * D) ** 2 / (cart_mass + M_POLE))
    return 2 * math.pi / math.sqrt(w2)


out = {"pole": {"m": M_POLE, "d": D, "I_com": round(I_COM, 7), "I_hinge": round(I_H, 7)}}
for name, M in (("turn1", 2.0), ("turn2", 0.5), ("turn3", None)):
    out[name] = {"cart_mass": M if M is not None else "locked",
                 "period_s": round(period(M), 4)}
print(json.dumps(out, indent=2))
