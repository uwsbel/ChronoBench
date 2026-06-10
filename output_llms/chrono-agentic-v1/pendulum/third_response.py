"""
Double Pendulum Simulation — PyChrono 9.0.x / Irrlicht

Models a double pendulum system consisting of two rigid arms connected in series:
  - Arm 1 (upper): attached to a fixed ground pivot via a revolute joint at its near end.
  - Arm 2 (lower): attached to the far end of Arm 1 via a second revolute joint.

Both arms swing independently under gravity (Y-up convention, gravity = -9.81 m/s²).
The system uses ChSystemNSC with no collision (pure jointed MBS).  Both pendulum arms
start offset from vertical so that interesting chaotic double-pendulum motion is visible.

Expected behavior: both arms swing freely, exhibiting the characteristic chaotic
motion of a double pendulum.  Arm 2 can spin and oscillate relative to Arm 1.
"""

# === Imports ===
import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters ===
# Physical constants
ARM1_LENGTH  = 1.0       # length of first pendulum arm [m]
ARM2_LENGTH  = 0.8       # length of second pendulum arm [m]
ARM_RADIUS   = 0.03      # visual cylinder radius for both arms [m]
ARM1_MASS    = 1.0       # mass of arm 1 [kg]
ARM2_MASS    = 0.8       # mass of arm 2 [kg]
PIN_RADIUS   = 0.04      # radius of pivot pin sphere [m]

# Initial conditions — arms displaced from vertical for chaotic motion
ARM1_INIT_ANGLE = math.radians(45.0)   # arm 1 initial angle from downward vertical [rad]
ARM2_INIT_ANGLE = math.radians(90.0)   # arm 2 initial angle from downward vertical [rad] (relative to arm1 direction)

# Simulation time
time_step = 1e-3          # physics timestep [s]
sim_end   = 15.0          # total simulation duration [s]
render_fps = 50.0         # frames per second for review video
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once: physics steps per frame

# Derived geometry (precomputed once)
# Pivot is at origin.  Arm-1 center is at half-length along its initial direction.
# Arm-2 connects at arm-1's far end; arm-2 center is half-length along arm-2's direction.
pivot_pos = chrono.ChVector3d(0.0, 0.0, 0.0)

# Arm 1: swings in XY plane.  Initial angle from the −Y axis (downward).
arm1_dir_x = math.sin(ARM1_INIT_ANGLE)
arm1_dir_y = -math.cos(ARM1_INIT_ANGLE)
arm1_center = chrono.ChVector3d(
    arm1_dir_x * ARM1_LENGTH / 2.0,
    arm1_dir_y * ARM1_LENGTH / 2.0,
    0.0,
)
arm1_far_end = chrono.ChVector3d(
    arm1_dir_x * ARM1_LENGTH,
    arm1_dir_y * ARM1_LENGTH,
    0.0,
)

# Arm 2: hangs from arm-1's far end, angle from downward vertical in world frame
arm2_world_angle = ARM1_INIT_ANGLE + ARM2_INIT_ANGLE
arm2_dir_x = math.sin(arm2_world_angle)
arm2_dir_y = -math.cos(arm2_world_angle)
arm2_center = chrono.ChVector3d(
    arm1_far_end.x + arm2_dir_x * ARM2_LENGTH / 2.0,
    arm1_far_end.y + arm2_dir_y * ARM2_LENGTH / 2.0,
    0.0,
)

# Inertia for rod about its own center (axis through Z, perpendicular to the rod)
# I_z = (1/12) * m * L^2  (thin rod rotating about center, perpendicular axis)
arm1_inertia_z = (1.0 / 12.0) * ARM1_MASS * ARM1_LENGTH ** 2
arm2_inertia_z = (1.0 / 12.0) * ARM2_MASS * ARM2_LENGTH ** 2

# === System & gravity ===
# ChSystemNSC — pure jointed MBS, no contact/collision needed (no SetCollisionSystemType)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))  # Y-up convention

# === Bodies ===
# --- Ground (fixed pivot) ---
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(pivot_pos)
ground.EnableCollision(False)
sys.AddBody(ground)

# Visual: sphere at ground pivot to show the attachment point
pin0_shape = chrono.ChVisualShapeSphere(PIN_RADIUS)
pin0_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(pin0_shape, chrono.ChFramed(pivot_pos))

# --- Arm 1 (upper pendulum) ---
# Body origin at arm-1's center; arm points from pivot to arm1_far_end.
# The arm swings in the XY plane; body-local X aligns with the arm direction.
arm1 = chrono.ChBody()
arm1.SetMass(ARM1_MASS)
arm1.SetInertiaXX(chrono.ChVector3d(arm1_inertia_z, arm1_inertia_z, arm1_inertia_z))
arm1.SetPos(arm1_center)
arm1.SetRot(chrono.QuatFromAngleZ(ARM1_INIT_ANGLE))   # body-local X along arm direction
arm1.EnableCollision(False)
sys.AddBody(arm1)

# Visual: cylinder along body-local X (two-step formula: SetRot above, then AddVisualShape with QuatFromAngleY offset)
cyl1_shape = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM1_LENGTH)
cyl1_shape.SetColor(chrono.ChColor(0.7, 0.2, 0.2))
arm1.AddVisualShape(cyl1_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Visual: sphere at arm-1 far end (hinge to arm-2)
pin1_shape = chrono.ChVisualShapeSphere(PIN_RADIUS)
pin1_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
arm1.AddVisualShape(pin1_shape, chrono.ChFramed(chrono.ChVector3d(ARM1_LENGTH / 2.0, 0.0, 0.0)))

# --- Arm 2 (lower pendulum) ---
arm2 = chrono.ChBody()
arm2.SetMass(ARM2_MASS)
arm2.SetInertiaXX(chrono.ChVector3d(arm2_inertia_z, arm2_inertia_z, arm2_inertia_z))
arm2.SetPos(arm2_center)
arm2.SetRot(chrono.QuatFromAngleZ(arm2_world_angle))  # body-local X along arm-2 direction
arm2.EnableCollision(False)
sys.AddBody(arm2)

# Visual: cylinder along body-local X
cyl2_shape = chrono.ChVisualShapeCylinder(ARM_RADIUS, ARM2_LENGTH)
cyl2_shape.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
arm2.AddVisualShape(cyl2_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Visual: sphere at arm-2 tip (bob)
bob_shape = chrono.ChVisualShapeSphere(PIN_RADIUS * 1.5)
bob_shape.SetColor(chrono.ChColor(0.9, 0.7, 0.1))
arm2.AddVisualShape(bob_shape, chrono.ChFramed(chrono.ChVector3d(ARM2_LENGTH / 2.0, 0.0, 0.0)))

# === Joints / constraints ===
# Revolute joint 1: ground (fixed) ↔ arm 1 at the ground pivot point.
# Hinge axis is world Z (motion in XY plane), so frame rotation = QUNIT (local +Z = world +Z).
hinge1 = chrono.ChLinkLockRevolute()
# 5-arg body-local form: frame on ground (world pos = pivot_pos), frame on arm1 near-end
hinge1.Initialize(
    ground, arm1,
    True,
    chrono.ChFramed(pivot_pos, chrono.QUNIT),                         # point on ground (world = pivot_pos since ground is at origin)
    chrono.ChFramed(chrono.ChVector3d(-ARM1_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),  # arm1 near-end (body-local)
)
sys.AddLink(hinge1)

# Revolute joint 2: arm 1 far-end ↔ arm 2 near-end.
# Both frames must map to the same world point = arm1_far_end.
hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(
    arm1, arm2,
    True,
    chrono.ChFramed(chrono.ChVector3d(+ARM1_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),  # arm1 far-end (body-local)
    chrono.ChFramed(chrono.ChVector3d(-ARM2_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),  # arm2 near-end (body-local)
)
sys.AddLink(hinge2)

# === Visualization ===
# Full Irrlicht block: Initialize() first, then scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Double Pendulum — PyChrono 9.0.x")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.0, -0.5, 4.0),   # eye: pulled back in Z, slightly above midpoint
    chrono.ChVector3d(0.0, -0.8, 0.0),   # look-at: center of pendulum swing
)
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5,
    20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -2.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Main loop ===


frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad configuration
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
