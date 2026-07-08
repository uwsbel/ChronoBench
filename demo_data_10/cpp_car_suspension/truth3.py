"""Suspension-car translation, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted
reference.

Same stiff-sprung car as turn 2, now DRIVEN: constant throttle 0.3 through the demo's own
simplified drivetrain (the translated ComputeWheelTorque law: differential averages the two rear
wheel speeds, conic and gearbox ratios 0.2 / 0.3, a DC-motor-like linear torque-speed curve with
80 Nm stall and 800 rad/s no-load, throttle-modulated, torque split to the two rear
ChLinkMotorRotationTorque links; recomputed every step as in the C++ render loop). The car
drops, settles, and accelerates along the drive direction. One deliberate deviation from the
C++: the ground plate grows 60 -> 400 m, because at throttle 0.3 the car outruns the demo's
plate inside the graded window (the calibration probe drove off the edge at ~73 m and fell; the
prompt states this change). The judge grades the settled ride, monotonic forward progress, and
the calibrated final speed; a candidate whose torque law is mis-translated misses the speed band.
"""
import csv
import json

import pychrono as chrono

STEP = 5e-3
T_END = 8.0
THROTTLE = 0.3
SPRING_K = 113200.0
SPRING_C = 80.0
CONIC_TAU = 0.2
GEAR_TAU = 0.3
MAX_MOTOR_TORQUE = 80.0
MAX_MOTOR_SPEED = 800.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravityY()
sysNSC.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetStaticFriction(1.0)
ground_mat.SetSlidingFriction(1.0)

ground = chrono.ChBodyEasyBox(400, 2, 400, 1.0, False, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetFixed(True)
sysNSC.AddBody(ground)

chassis_mat = chrono.ChContactMaterialNSC()
wheel_mat = chrono.ChContactMaterialNSC()
wheel_mat.SetFriction(1.0)

chassis = chrono.ChBodyEasyBox(1, 0.5, 3, 1.0, False, True, chassis_mat)
chassis.SetPos(chrono.ChVector3d(0, 1, 0))
chassis.SetMass(150)
chassis.SetInertiaXX(chrono.ChVector3d(4.8, 4.5, 1))
chassis.SetFixed(False)
sysNSC.AddBody(chassis)

springs = []
motors = []


def build_corner(sx, sz, front):
    """One suspension corner the demo's way; sx = +-1 (right/left), sz = +-1 (front/back)."""
    spindle = chrono.ChBodyEasyBox(0.1, 0.4, 0.4, 1.0, False, False)
    spindle.SetPos(chrono.ChVector3d(1.3 * sx, 1, 1 * sz))
    spindle.SetMass(8)
    spindle.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
    sysNSC.AddBody(spindle)

    wheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.45, 0.3, 1.0, False, True, wheel_mat)
    wheel.SetPos(chrono.ChVector3d(1.5 * sx, 1, 1 * sz))
    wheel.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))
    wheel.SetMass(3)
    wheel.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
    sysNSC.AddBody(wheel)

    rev = chrono.ChLinkLockRevolute()
    rev.Initialize(wheel, spindle, chrono.ChFramed(chrono.ChVector3d(1.5 * sx, 1, 1 * sz),
                                                   chrono.QuatFromAngleY(chrono.CH_PI_2)))
    sysNSC.AddLink(rev)

    for cy, cz in ((1.2, 0.2), (1.2, -0.2), (0.8, 0.2), (0.8, -0.2)):
        rod = chrono.ChLinkDistance()
        rod.Initialize(chassis, spindle, False,
                       chrono.ChVector3d(0.5 * sx, cy, (1 + cz) * sz),
                       chrono.ChVector3d(1.25 * sx, cy, 1 * sz))
        sysNSC.AddLink(rod)

    spring = chrono.ChLinkTSDA()
    spring.Initialize(chassis, spindle, False,
                      chrono.ChVector3d(0.5 * sx, 1.2, 1.0 * sz),
                      chrono.ChVector3d(1.25 * sx, 0.8, 1 * sz))
    spring.SetSpringCoefficient(SPRING_K)
    spring.SetDampingCoefficient(SPRING_C)
    sysNSC.AddLink(spring)
    springs.append(spring)

    steer = chrono.ChLinkDistance()
    steer.Initialize(chassis, spindle, False,
                     chrono.ChVector3d(0.5 * sx, 1.21, 1.4 * sz),
                     chrono.ChVector3d(1.25 * sx, 1.21, 1.3 * sz))
    sysNSC.AddLink(steer)

    if not front:
        motor = chrono.ChLinkMotorRotationTorque()
        motor.Initialize(wheel, chassis, chrono.ChFramed(chrono.ChVector3d(1.5 * sx, 1, 1 * sz),
                                                         chrono.QuatFromAngleY(chrono.CH_PI_2)))
        sysNSC.AddLink(motor)
        motors.append(motor)

    return spindle, wheel


build_corner(+1, +1, True)    # right front
build_corner(-1, +1, True)    # left front
build_corner(+1, -1, False)   # right back
build_corner(-1, -1, False)   # left back


def compute_wheel_torque():
    """The demo's simplified throttle -> wheel-torque law (differential + gear + DC-like motor)."""
    shaftspeed = (1.0 / CONIC_TAU) * 0.5 * (motors[0].GetMotorAngleDt() + motors[1].GetMotorAngleDt())
    motorspeed = (1.0 / GEAR_TAU) * shaftspeed
    motortorque = (MAX_MOTOR_TORQUE - motorspeed * (MAX_MOTOR_TORQUE / MAX_MOTOR_SPEED)) * THROTTLE
    shafttorque = motortorque * (1.0 / GEAR_TAU)
    single = 0.5 * shafttorque * (1.0 / CONIC_TAU)
    for m in motors:
        m.SetTorqueFunction(chrono.ChFunctionConst(single))


sysNSC.SetSolverType(chrono.ChSolver.Type_PSOR)
sysNSC.GetSolver().AsIterative().SetMaxIterations(20)

z0 = chassis.GetPos().z
rows = []
t = 0.0
while t < T_END:
    t = sysNSC.GetChTime()
    compute_wheel_torque()
    sysNSC.DoStepDynamics(STEP)
    p = chassis.GetPos()
    v = chassis.GetPosDt()
    rows.append((t, p.y, springs[0].GetLength(), abs(p.z - z0), v.Length()))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "cy", "slen", "dist", "speed"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}", f"{r[4]:.6e}"])

tail = [r for r in rows if r[0] >= T_END - 1.0]
print(json.dumps({"chassis_y_settled": sum(r[1] for r in tail) / len(tail),
                  "spring_len_settled": sum(r[2] for r in tail) / len(tail),
                  "dist_final": rows[-1][3],
                  "speed_final": rows[-1][4],
                  "throttle": THROTTLE}))
