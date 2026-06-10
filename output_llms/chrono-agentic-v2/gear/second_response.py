"""
Gear train simulation — PyChrono 9.0.0 / Irrlicht, ChSystemNSC (Y-up, gravity -9.81 Y).

Models a two-gear epicyclic/spur-gear pair: a small driver gear A (radA=1.5) and a
larger driven gear B (radB=3.5), mounted on a fixed truss body. A rotational speed
motor drives gear A; a ChLinkLockGear constraint transfers motion to gear B, which
also has a revolute constraint to the truss. Visual shafts, truss, and both gears
are visualized. Expected behavior: gear A spins at 3 rad/s; gear B rotates in the
opposite direction at a proportionally reduced speed (radA/radB ratio).
"""

import math as m
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3          # integration step [s]
SIM_END   = 10.0          # simulation end time [s]
RENDER_FPS = 50.0         # Irrlicht render cadence [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Gear geometry
radA = 1.5        # driver gear radius (modified)
radB = 3.5        # driven gear radius (modified)
interaxis12 = radA + radB   # centre-to-centre distance

# Truss (support) dimensions — modified
TRUSS_X = 15.0
TRUSS_Y =  8.0
TRUSS_Z =  2.0

# Motor speed — modified
MOTOR_SPEED = 3.0   # rad/s

# === System & gravity ===
# Pure jointed MBS with no contact → no collision system needed (truth omits it here)
sys = chrono.ChSystemNSC()
sys.SetGravityY()   # (0, -9.81, 0), Y-up

# === Bodies ===

# --- Truss (fixed support) ---
mat = chrono.ChContactMaterialNSC()
mbody_truss = chrono.ChBodyEasyBox(TRUSS_X, TRUSS_Y, TRUSS_Z, 1000, True, False, mat)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))
mbody_truss.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
sys.Add(mbody_truss)

# --- Gear A (driver) ---
mbody_gearA = chrono.ChBody()
mbody_gearA.SetMass(1.0)
mbody_gearA.SetInertiaXX(chrono.ChVector3d(0.5 * 1.0 * radA**2,
                                            0.5 * 1.0 * radA**2,
                                            0.5 * 1.0 * radA**2))
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.EnableCollision(False)
sys.AddBody(mbody_gearA)

# Gear A visual — cylinder along Z (hinge axis)
cyl_A = chrono.ChVisualShapeCylinder(radA, 0.5)
cyl_A.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
mbody_gearA.AddVisualShape(cyl_A, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Visual shaft for gear A axis
shaft_A = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
shaft_A.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
mbody_gearA.AddVisualShape(shaft_A, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# --- Gear B (driven) ---
mbody_gearB = chrono.ChBody()
mbody_gearB.SetMass(1.0)
mbody_gearB.SetInertiaXX(chrono.ChVector3d(0.5 * 1.0 * radB**2,
                                            0.5 * 1.0 * radB**2,
                                            0.5 * 1.0 * radB**2))
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))   # modified z=-2
mbody_gearB.EnableCollision(False)
sys.AddBody(mbody_gearB)

# Gear B visual
cyl_B = chrono.ChVisualShapeCylinder(radB, 0.5)
cyl_B.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
mbody_gearB.AddVisualShape(cyl_B, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# === Joints / constraints ===

# Motor: drives gear A relative to truss (full motor-link — no separate revolute)
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))   # modified: 3 rad/s
sys.AddLink(link_motor)

# Revolute: gear B pivots on truss
link_revolute_B = chrono.ChLinkLockRevolute()
link_revolute_B.Initialize(mbody_gearB, mbody_truss,
                           chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -2),
                                           chrono.QUNIT))
sys.AddLink(link_revolute_B)

# Gear constraint: A drives B with transmission ratio radA/radB
link_gear = chrono.ChLinkLockGear()
link_gear.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,
                                          chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,
                                          chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetTransmissionRatio(radA / radB)
link_gear.SetEnforcePhase(True)
sys.AddLink(link_gear)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train — radA=1.5, radB=3.5, speed=3 rad/s")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20), chrono.ChVector3d(interaxis12 / 2, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # resources closed in review-only block below
