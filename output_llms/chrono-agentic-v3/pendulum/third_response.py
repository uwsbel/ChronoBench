"""
Double pendulum simulation using PyChrono (ChSystemNSC, Y-up).

System: ChSystemNSC, gravity (0, -9.81, 0).
Bodies:
  - floor: fixed anchor at the world origin
  - arm1: first pendulum rod, pivots at its near end attached to floor
  - arm2: second pendulum rod, pivots at its near end attached to arm1's far end
Joints:
  - hinge1: revolute between arm1 and floor at the pivot point (world origin)
  - hinge2: revolute between arm2 and arm1 at arm1's far end
Expected: chaotic double-pendulum motion where both arms swing under gravity,
demonstrating independent rotation of each link about its respective hinge.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP    = 1e-3           # physics timestep [s]
SIM_END      = 10.0           # simulation end time [s]
RENDER_FPS   = 50.0           # render cadence [Hz]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

ARM_LENGTH   = 1.0            # length of each pendulum arm [m]
ARM_RADIUS   = 0.04           # rod visual radius [m]
ARM_MASS     = 1.0            # mass of each arm [kg]
ARM_INERTIA  = 0.2            # principal inertia [kg·m²]

# Initial angles from downward vertical (Y-down), measured in XY plane
ARM1_ANGLE   = math.pi / 4.0        # arm1: 45 deg from vertical [rad]
ARM2_REL_ANGLE = math.pi / 3.0      # arm2 offset relative to arm1 [rad]
ARM2_ANGLE   = ARM1_ANGLE + ARM2_REL_ANGLE  # arm2 absolute angle [rad]

# Pivot 1 is at world origin (0, 0, 0)
PIVOT1 = chrono.ChVector3d(0, 0, 0)

# Direction vectors: angle from downward vertical => dir = (sin(A), -cos(A), 0)
arm1_dir_x = math.sin(ARM1_ANGLE)
arm1_dir_y = -math.cos(ARM1_ANGLE)

# arm1 COM and far end (pivot 2) — precomputed once
ARM1_COM = chrono.ChVector3d(
    PIVOT1.x + 0.5 * ARM_LENGTH * arm1_dir_x,
    PIVOT1.y + 0.5 * ARM_LENGTH * arm1_dir_y,
    0
)
PIVOT2 = chrono.ChVector3d(
    PIVOT1.x + ARM_LENGTH * arm1_dir_x,
    PIVOT1.y + ARM_LENGTH * arm1_dir_y,
    0
)

arm2_dir_x = math.sin(ARM2_ANGLE)
arm2_dir_y = -math.cos(ARM2_ANGLE)
ARM2_COM = chrono.ChVector3d(
    PIVOT2.x + 0.5 * ARM_LENGTH * arm2_dir_x,
    PIVOT2.y + 0.5 * ARM_LENGTH * arm2_dir_y,
    0
)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS with no contact — SetCollisionSystemType omitted

# === Bodies ===
# Fixed anchor (floor pivot)
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
pivot_vis = chrono.ChVisualShapeSphere(0.06)
pivot_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
floor.AddVisualShape(pivot_vis)
sys.AddBody(floor)

# Arm 1 (first pendulum rod)
# Body origin at COM; body-local X axis points along the rod direction.
# arm1_dir is at angle ARM1_ANGLE from downward, i.e. at angle (ARM1_ANGLE - pi/2) from +X.
arm1 = chrono.ChBody()
arm1.SetMass(ARM_MASS)
arm1.SetInertiaXX(chrono.ChVector3d(ARM_INERTIA, ARM_INERTIA, ARM_INERTIA))
arm1.SetPos(ARM1_COM)
arm1.SetRot(chrono.QuatFromAngleZ(ARM1_ANGLE - math.pi / 2.0))
arm1.EnableCollision(False)
cyl1 = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LENGTH)
cyl1.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
arm1.AddVisualShape(cyl1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
# Small visual marker at arm1's far end (= pivot 2)
hinge2_marker = chrono.ChVisualShapeSphere(0.05)
hinge2_marker.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
arm1.AddVisualShape(hinge2_marker, chrono.ChFramed(
    chrono.ChVector3d(ARM_LENGTH / 2, 0, 0), chrono.QUNIT))
sys.AddBody(arm1)

# Arm 2 (second pendulum rod)
arm2 = chrono.ChBody()
arm2.SetMass(ARM_MASS)
arm2.SetInertiaXX(chrono.ChVector3d(ARM_INERTIA, ARM_INERTIA, ARM_INERTIA))
arm2.SetPos(ARM2_COM)
arm2.SetRot(chrono.QuatFromAngleZ(ARM2_ANGLE - math.pi / 2.0))
arm2.EnableCollision(False)
cyl2 = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM_LENGTH)
cyl2.SetColor(chrono.ChColor(0.1, 0.4, 0.8))
arm2.AddVisualShape(cyl2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(arm2)

# === Joints / constraints ===
# hinge1: arm1 near-end ↔ floor at world origin
# Body-local form: arm1 near end at (-ARM_LENGTH/2, 0, 0) in arm1's local frame;
# floor anchor at (0, 0, 0) in floor's local frame.
# Hinge axis is world Z (XY-plane swing) => frame rotation = QUNIT.
hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(
    arm1, floor, True,
    chrono.ChFramed(chrono.ChVector3d(-ARM_LENGTH / 2, 0, 0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
)
sys.AddLink(hinge1)

# hinge2: arm2 near-end ↔ arm1 far-end
# arm1 far end at (+ARM_LENGTH/2, 0, 0) in arm1's local frame;
# arm2 near end at (-ARM_LENGTH/2, 0, 0) in arm2's local frame.
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm2, arm1, True,
    chrono.ChFramed(chrono.ChVector3d(-ARM_LENGTH / 2, 0, 0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(+ARM_LENGTH / 2, 0, 0), chrono.QUNIT),
)
sys.AddLink(hinge2)

# === Visualization ===  full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum — PyChrono 9.0.x")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1, 5), chrono.ChVector3d(0, -1, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Review-only setup ===


# === Main loop ===
while vis.Run() and sys.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(TIME_STEP)
        if sys.GetChTime() >= SIM_END:
            break
