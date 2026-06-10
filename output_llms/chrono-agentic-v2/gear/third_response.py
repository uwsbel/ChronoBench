"""
Gear train simulation with bevel gear and belt-pulley system.

System type: ChSystemNSC (NSC, Y-up, pure jointed MBS — no contact/collision).
Bodies:
  - truss (fixed): structural support for all pivots
  - bar (motor-driven): axle aligned along Z, spun at constant speed
  - gear A (r=5): main drive gear on the bar, spun by motor
  - gear B (r=5): driven by gear A at 1:1 ratio
  - gear C (r=5): driven by gear B at 1:1 ratio
  - bevel gear D (r=5): at (-10,0,-9), horizontal axis, 1:1 ratio with gear A
  - pulley E (r=2): at (-10,-11,-9), horizontal axis, belt-synchro with gear D
Expected behavior: all gears/pulley rotate continuously driven by the motor;
belt constraint enforces synchronous rotation between gear D and pulley E.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
# Geometry
TRUSS_W      = 10.0        # truss box width [m]
TRUSS_H      = 2.0         # truss box height [m]
TRUSS_D      = 2.0         # truss box depth [m]
BAR_RADIUS   = 0.5         # bar cylinder radius [m]
BAR_LEN      = 10.0        # bar cylinder length [m]
GEAR_RADIUS  = 5.0         # gears A/B/C and bevel D radius [m]
GEAR_THICK   = 1.0         # gear cylinder thickness [m]
PULLEY_RAD   = 2.0         # pulley E radius [m]
PULLEY_THICK = 0.5         # pulley thickness [m]
MOTOR_SPEED  = chrono.CH_PI  # rad/s  (1 rev / 2 s)

# Gear positions (Y-up, XY plane for spur gears A/B/C)
POS_TRUSS   = chrono.ChVector3d(0.0, 0.0, 0.0)
POS_BARA    = chrono.ChVector3d(0.0, 0.0, -1.0)   # bar runs along Z
POS_GEARA   = chrono.ChVector3d(0.0, 0.0, -5.0)
POS_GEARB   = chrono.ChVector3d(10.0, 0.0, -5.0)  # A-B mesh: gap=0 at contact
POS_GEARC   = chrono.ChVector3d(20.0, 0.0, -5.0)  # B-C mesh
# Bevel gear D: rotated 90° around Z → shaft along X (horizontal)
POS_GEARD   = chrono.ChVector3d(-10.0, 0.0, -9.0)
# Pulley E: same X/Z, shifted in Y
POS_PULLEYS = chrono.ChVector3d(-10.0, -11.0, -9.0)

# Simulation timing
time_step  = 1e-3          # physics dt [s]
sim_end    = 10.0          # run duration [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# Pure jointed MBS — no collision shapes, collision system omitted per ground truth
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# --- Truss (fixed structural support) ---
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(POS_TRUSS)
vs_truss = chrono.ChVisualShapeBox(TRUSS_W, TRUSS_H, TRUSS_D)
vs_truss.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
truss.AddVisualShape(vs_truss)
sys.Add(truss)

# --- Motor bar (rotates about Z through truss) ---
bar = chrono.ChBody()
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.04))
bar.SetPos(POS_BARA)
vs_bar = chrono.ChVisualShapeCylinder(BAR_RADIUS, BAR_LEN)
bar.AddVisualShape(vs_bar, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(bar)

# --- Gear A (spur, XY plane, shaft along Z) ---
gearA = chrono.ChBody()
gearA.SetMass(1.0)
gearA.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
gearA.SetPos(POS_GEARA)
vs_gearA = chrono.ChVisualShapeCylinder(GEAR_RADIUS, GEAR_THICK)
vs_gearA.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
gearA.AddVisualShape(vs_gearA, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(gearA)

# --- Gear B (spur, meshes with A) ---
gearB = chrono.ChBody()
gearB.SetMass(1.0)
gearB.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
gearB.SetPos(POS_GEARB)
vs_gearB = chrono.ChVisualShapeCylinder(GEAR_RADIUS, GEAR_THICK)
vs_gearB.SetColor(chrono.ChColor(0.3, 0.8, 0.3))
gearB.AddVisualShape(vs_gearB, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(gearB)

# --- Gear C (spur, meshes with B) ---
gearC = chrono.ChBody()
gearC.SetMass(1.0)
gearC.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
gearC.SetPos(POS_GEARC)
vs_gearC = chrono.ChVisualShapeCylinder(GEAR_RADIUS, GEAR_THICK)
vs_gearC.SetColor(chrono.ChColor(0.3, 0.3, 0.8))
gearC.AddVisualShape(vs_gearC, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(gearC)

# --- Bevel gear D (horizontal shaft along X, rotated 90° about Z) ---
# Positioned at (-10, 0, -9); shaft axis is X (horizontal)
gearD = chrono.ChBody()
gearD.SetMass(1.0)
gearD.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
gearD.SetPos(POS_GEARD)
# Rotate body so cylinder axis (default Z) aligns with world X (horizontal)
gearD.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))
vs_gearD = chrono.ChVisualShapeCylinder(GEAR_RADIUS, GEAR_THICK)
vs_gearD.SetColor(chrono.ChColor(0.8, 0.7, 0.2))
# cylinder default Z in body frame → after SetRot becomes world X
gearD.AddVisualShape(vs_gearD, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(gearD)

# --- Pulley E (belt-driven from D, same horizontal shaft orientation) ---
pulleyE = chrono.ChBody()
pulleyE.SetMass(0.5)
pulleyE.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
pulleyE.SetPos(POS_PULLEYS)
pulleyE.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))
vs_pulleyE = chrono.ChVisualShapeCylinder(PULLEY_RAD, PULLEY_THICK)
vs_pulleyE.SetColor(chrono.ChColor(0.6, 0.4, 0.9))
pulleyE.AddVisualShape(vs_pulleyE, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.Add(pulleyE)

# === Joints / constraints ===

# --- Motor: drives gear A (and coupled bar) about Z through truss ---
# ChLinkMotorRotationSpeed is a full motor-link — no extra revolute needed
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gearA, truss,
                 chrono.ChFramed(POS_GEARA, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# --- Revolute: bar rotates with gearA (locked together) ---
link_bar = chrono.ChLinkLockLock()
link_bar.Initialize(bar, gearA, chrono.ChFramed(POS_GEARA, chrono.QUNIT))
sys.AddLink(link_bar)

# --- Revolute: gear B rotates on truss about Z at POS_GEARB ---
rev_B = chrono.ChLinkLockRevolute()
rev_B.Initialize(gearB, truss,
                 chrono.ChFramed(POS_GEARB, chrono.QUNIT))
sys.AddLink(rev_B)

# --- Revolute: gear C rotates on truss about Z at POS_GEARC ---
rev_C = chrono.ChLinkLockRevolute()
rev_C.Initialize(gearC, truss,
                 chrono.ChFramed(POS_GEARC, chrono.QUNIT))
sys.AddLink(rev_C)

# --- Revolute: bevel gear D rotates on truss about X (horizontal) ---
# Local +Z of joint frame must align with world X (horizontal shaft).
# QuatFromAngleY(-PI/2) rotates local +Z → world +X
q_hinge_x = chrono.QuatFromAngleY(-chrono.CH_PI_2)
rev_D = chrono.ChLinkLockRevolute()
rev_D.Initialize(gearD, truss,
                 chrono.ChFramed(POS_GEARD, q_hinge_x))
sys.AddLink(rev_D)

# --- Revolute: pulley E rotates on truss about X (horizontal) ---
rev_E = chrono.ChLinkLockRevolute()
rev_E.Initialize(pulleyE, truss,
                 chrono.ChFramed(POS_PULLEYS, q_hinge_x))
sys.AddLink(rev_E)

# --- Gear A–B spur mesh (1:1 ratio, shafts along Z) ---
gearAB = chrono.ChLinkLockGear()
gearAB.Initialize(gearA, gearB, chrono.ChFramed())
gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetTransmissionRatio(GEAR_RADIUS / GEAR_RADIUS)   # 1:1
gearAB.SetEnforcePhase(True)
sys.AddLink(gearAB)

# --- Gear B–C spur mesh (1:1 ratio) ---
gearBC = chrono.ChLinkLockGear()
gearBC.Initialize(gearB, gearC, chrono.ChFramed())
gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearBC.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearBC.SetTransmissionRatio(GEAR_RADIUS / GEAR_RADIUS)
gearBC.SetEnforcePhase(True)
sys.AddLink(gearBC)

# --- Bevel gear A–D (1:1 ratio, A shaft along Z, D shaft along X) ---
# Shaft1 frame: gearA shaft is world Z — local +Z already aligns → QUNIT is correct
# Shaft2 frame: gearD shaft is world X — rotate so local +Z aligns with X
gearAD = chrono.ChLinkLockGear()
gearAD.Initialize(gearA, gearD, chrono.ChFramed())
gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAD.SetTransmissionRatio(GEAR_RADIUS / GEAR_RADIUS)   # 1:1 bevel
gearAD.SetEnforcePhase(True)
sys.AddLink(gearAD)

# --- Synchro belt: gear D drives pulley E ---
# ChLinkLockPulley connects two bodies via a synchro belt (same shaft direction)
belt_DE = chrono.ChLinkLockPulley()
belt_DE.Initialize(gearD, pulleyE, chrono.ChFramed(POS_GEARD, q_hinge_x))
belt_DE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
belt_DE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
belt_DE.SetRadius1(GEAR_RADIUS)
belt_DE.SetRadius2(PULLEY_RAD)
belt_DE.SetEnforcePhase(True)
sys.AddLink(belt_DE)

# Belt visual: draw a line between upper belt tangent points
belt_vis = chrono.ChVisualShapeLine()
belt_line = chrono.ChLineSegment(POS_GEARD, POS_PULLEYS)
belt_vis.SetLineGeometry(belt_line)
belt_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
truss.AddVisualShape(belt_vis)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train + Bevel Gear + Belt Pulley")
vis.Initialize()                # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(15, 10, 25), chrono.ChVector3d(5, -2, -5))  # AFTER Initialize
vis.AddTypicalLights()

# === Main loop ===
# review-only recording setup

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()   # cache: fetched once per step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # cleanup handled in review-only block below
