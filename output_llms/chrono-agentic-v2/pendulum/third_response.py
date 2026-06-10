"""
Double pendulum simulation using PyChrono 9.0.x with Irrlicht visualization.

System type: ChSystemNSC (Y-up, gravity = (0, -9.81, 0))
Bodies:
  - ground: fixed support body
  - arm1: first pendulum arm pivoting about the ground pin
  - arm2: second pendulum arm pivoting about the tip of arm1
Joints:
  - hinge1: ChLinkLockRevolute connecting ground to arm1 (pivot at origin)
  - hinge2: ChLinkLockRevolute connecting arm1 tip to arm2 base
Expected behavior: chaotic double-pendulum motion; both arms swing
  independently about their respective hinge axes.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3         # physics timestep [s]
SIM_END   = 10.0         # simulation duration [s]
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

ARM1_MASS   = 1.0        # kg
ARM1_LENGTH = 1.0        # m
ARM1_RADIUS = 0.04       # visual cylinder radius

ARM2_MASS   = 0.5        # kg
ARM2_LENGTH = 0.8        # m
ARM2_RADIUS = 0.03       # visual cylinder radius

# Hinge positions in world frame (Y-up)
HINGE1_POS = chrono.ChVector3d(0, 2.0, 0)   # fixed pivot at top
# arm1 hangs from HINGE1; its COM is at hinge1 - ARM1_LENGTH/2 along Y
ARM1_POS   = chrono.ChVector3d(0, HINGE1_POS.y - ARM1_LENGTH / 2.0, 0)
# arm1 tip -> hinge2 is at hinge1 - ARM1_LENGTH along Y
HINGE2_POS = chrono.ChVector3d(0, HINGE1_POS.y - ARM1_LENGTH, 0)
# arm2 COM is at hinge2 - ARM2_LENGTH/2 along Y (initial vertical)
ARM2_POS   = chrono.ChVector3d(0, HINGE2_POS.y - ARM2_LENGTH / 2.0, 0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravityY()   # (0, -9.81, 0)
# Pure jointed MBS with no contact — collision system omitted per rules

# === Bodies ===

# Ground: fixed support body (small box at pivot height)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(HINGE1_POS)
ground_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
ground_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_shape)
sys.AddBody(ground)

# arm1: first pendulum arm
arm1 = chrono.ChBody()
arm1.SetMass(ARM1_MASS)
arm1.SetInertiaXX(chrono.ChVector3d(0.2, 1e-4, 0.2))
arm1.SetPos(ARM1_POS)
# Arm hangs along -Y: body-local X will be the arm axis after SetRot
# Cylinder default axis is local Z; QuatFromAngleY(PI/2) rotates Z->X;
# QuatFromAngleZ(-PI/2) rotates local X to point down (-Y in world).
arm1.SetRot(chrono.QuatFromAngleZ(-math.pi / 2.0))
cyl1 = chrono.ChVisualShapeCylinder(ARM1_RADIUS, ARM1_LENGTH)
cyl1.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
arm1.AddVisualShape(cyl1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(arm1)

# arm2: second pendulum arm
arm2 = chrono.ChBody()
arm2.SetMass(ARM2_MASS)
arm2.SetInertiaXX(chrono.ChVector3d(0.1, 1e-4, 0.1))
arm2.SetPos(ARM2_POS)
arm2.SetRot(chrono.QuatFromAngleZ(-math.pi / 2.0))
cyl2 = chrono.ChVisualShapeCylinder(ARM2_RADIUS, ARM2_LENGTH)
cyl2.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
arm2.AddVisualShape(cyl2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(arm2)

# Give arm2 a small initial angular nudge so the system starts in chaotic motion
arm2.SetAngVelParent(chrono.ChVector3d(0, 0, 0.5))

# === Joints / constraints ===
# hinge1: ground <-> arm1; pivot at HINGE1_POS; hinge axis = world +Z (QUNIT)
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(
    arm1, ground, True,
    # arm1 local: near end = -ARM1_LENGTH/2 along body-local X
    chrono.ChFramed(chrono.ChVector3d(-ARM1_LENGTH / 2.0, 0, 0), chrono.QUNIT),
    # ground local: arm1 pivot relative to ground center (ground is at HINGE1_POS)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
)
sys.AddLink(hinge1)

# hinge2: arm1 <-> arm2; pivot at the far end of arm1 / near end of arm2
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm1, arm2, True,
    # arm1 local: far end = +ARM1_LENGTH/2 along body-local X
    chrono.ChFramed(chrono.ChVector3d(+ARM1_LENGTH / 2.0, 0, 0), chrono.QUNIT),
    # arm2 local: near end = -ARM2_LENGTH/2 along body-local X
    chrono.ChFramed(chrono.ChVector3d(-ARM2_LENGTH / 2.0, 0, 0), chrono.QUNIT),
)
sys.AddLink(hinge2)

# === Visualization ===  full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum — PyChrono 9.0.0")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(0, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
