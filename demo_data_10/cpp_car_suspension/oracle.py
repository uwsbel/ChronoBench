"""Independent oracle for the cpp_car_suspension task (stdlib math only, NO Chrono).

The suspension-settle physics of the MySimpleCar (demo_MBS_suspension.cpp): each corner's
spindle is held to the chassis by four horizontal wishbone rods plus a steer/lateral rod, so its
one free motion relative to the chassis is (near-)vertical, and the angled spring-damper
(chassis anchor (0.5, 1.2, 1.0) to spindle anchor (1.25, 0.8, 1.0), rest length
|(0.75, -0.4, 0)| = 0.85) resists that motion through a geometric MOTION RATIO: a unit vertical
spindle drop lengthens the spring by only dy/L = 0.4/0.85 = 0.4706.

Linearized static force balance per corner (chassis weight only; the spindle and wheel weights
pass through the rods and contact, not the spring):

    k * dl * (dy/L) = m_chassis * g / 4
    dl = (150 * 9.81 / 4) / (k * 0.4706)

  turn 1 (translate): k = 28300  -> dl = 0.0276 m (spring settles at 0.8224).
                      MEASURED on the pinned build: 0.0276 (matches the linearization to 4 decimals).
  turn 2 (modify):    k = 113200 -> dl = 0.0069 m (spring settles at 0.8431).
                      MEASURED: 0.0071 (3%). The suspension law: 4x stiffer, ~1/4 the
                      compression; the chassis rides HIGHER (measured 0.4205 -> 0.4422).
  turn 3 (extend):    driven at throttle 0.3 through the demo's DC-motor-like drivetrain
                      (stall wheel torque 0.5 * (0.3*80/0.3) / 0.2 = 200 Nm per rear wheel,
                      no-load wheel speed 800 * 0.3 * 0.2 = 48 rad/s -> 21.6 m/s terminal at
                      wheel radius 0.45). MEASURED: 81.13 m and 17.01 m/s after 8 s
                      (approaching terminal), ride unchanged from turn 2.

The chassis ride HEIGHT couples the spring compression to the wheel-contact geometry
(wheel center at the 0.45 tire radius, minus the suspension drop dl / (dy/L)); the rigid
estimate lands within ~3 cm of the measured settle, so the ride-height bands are
calibrated-and-frozen around the measured values while the SPRING COMPRESSION carries the
independent force-balance anchor (2-3%). Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/cpp_car_suspension/oracle.py
"""
import json

G = 9.81
M_CHASSIS = 150.0
CORNER_LOAD = M_CHASSIS * G / 4
MOTION_RATIO = 0.4 / 0.85
REST_LEN = 0.85

out = {}
for name, k in (("turn1", 28300.0), ("turn2", 113200.0), ("turn3", 113200.0)):
    dl = CORNER_LOAD / (k * MOTION_RATIO)
    out[name] = {"k": k, "corner_load_N": round(CORNER_LOAD, 1),
                 "spring_compression_pred": round(dl, 4),
                 "spring_len_pred": round(REST_LEN - dl, 4)}
out["turn3"]["drive"] = {"throttle": 0.3, "stall_wheel_torque_Nm": 200.0,
                         "terminal_speed_mps": round(800 * 0.3 * 0.2 * 0.45, 1)}
print(json.dumps(out, indent=2))
