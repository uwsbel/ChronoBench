"""
Simple Pendulum Simulation using PyChrono with Irrlicht visualization.

System type: ChSystemNSC (Non-Smooth Contact, Y-up gravity)
Bodies:
  - ground: fixed body at origin serving as the pivot support
  - pendulum: rod-like arm (mass=1 kg, length=1 m) hung from a revolute joint
    at the top, free to swing in the XY plane
Constraints:
  - Revolute joint at origin (pivot point) connecting ground to pendulum,
    hinge axis along world Z (standard XY-plane pendulum)
Expected behavior:
  - Pendulum starts displaced ~45 degrees from vertical and swings freely under
    gravity, exhibiting classic sinusoidal oscillation with conserved energy
    (no damping, pure kinematic constraint).
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Physics parameters
PENDULUM_MASS = 1.0          # kg
PENDULUM_LENGTH = 1.0        # m  (pivot to bob center)
PENDULUM_RADIUS = 0.02       # m  (arm visual radius)
BOB_RADIUS = 0.05            # m  (bob sphere radius)
INITIAL_ANGLE_DEG = 45.0     # degrees from vertical (XY plane swing)
INITIAL_ANGLE_RAD = math.radians(INITIAL_ANGLE_DEG)

# Pendulum moment of inertia about pivot = m*L^2 (treating as point mass at end)
# For a homogeneous rod: I_cm = (1/12)*m*L^2; I_pivot = I_cm + m*(L/2)^2 = (1/3)*m*L^2
PENDULUM_INERTIA = (1.0 / 3.0) * PENDULUM_MASS * PENDULUM_LENGTH ** 2  # kg·m² about pivot

# Center-of-mass of pendulum is at L/2 below pivot
PENDULUM_COM_X = (PENDULUM_LENGTH / 2.0) * math.sin(INITIAL_ANGLE_RAD)
PENDULUM_COM_Y = -(PENDULUM_LENGTH / 2.0) * math.cos(INITIAL_ANGLE_RAD)

# Simulation timing
TIME_STEP = 1e-3   # s
SIM_END = 10.0     # s
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemNSC: rigid-impulsive contact; gravity along -Y (Y-up MBS convention)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# No SetCollisionSystemType: pure jointed MBS with no contact

# === Bodies ===
# Ground body — fixed support at the origin
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
# Visual: small box representing the pivot mount
ground_box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
ground_box.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(ground_box)
sys.AddBody(ground)

# Pendulum body — rod + bob at end, modeled with COM at L/2 below pivot
pendulum = chrono.ChBody()
pendulum.SetMass(PENDULUM_MASS)
# Inertia about body COM (same as rod formula about COM, then adjusted for local use)
# Using uniform rod: I_cm_zz = (1/12)*m*L^2; the hinge at COM offset is handled by the joint
I_cm = (1.0 / 12.0) * PENDULUM_MASS * PENDULUM_LENGTH ** 2
pendulum.SetInertiaXX(chrono.ChVector3d(I_cm, I_cm, I_cm))
pendulum.SetPos(chrono.ChVector3d(PENDULUM_COM_X, PENDULUM_COM_Y, 0.0))
pendulum.SetRot(chrono.QuatFromAngleZ(INITIAL_ANGLE_RAD))  # initial tilt
pendulum.EnableCollision(False)
sys.AddBody(pendulum)

# Visual: cylinder arm aligned along body-local X (the rod axis),
# Step 1: body rotation already set by SetRot(QuatFromAngleZ(theta)) above
# Step 2: visual offset rotates cylinder Z -> X with QuatFromAngleY(CH_PI_2)
arm_shape = chrono.ChVisualShapeCylinder(PENDULUM_RADIUS, PENDULUM_LENGTH)
arm_shape.SetColor(chrono.ChColor(0.8, 0.5, 0.2))
pendulum.AddVisualShape(
    arm_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))
)

# Visual: sphere (bob) at the far end of the arm in body-local frame
bob_offset_x = PENDULUM_LENGTH / 2.0  # COM is at center; bob at +L/2 in body-local X
bob_shape = chrono.ChVisualShapeSphere(BOB_RADIUS)
bob_shape.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
pendulum.AddVisualShape(bob_shape, chrono.ChFramed(chrono.ChVector3d(bob_offset_x, 0, 0)))

# === Joints / constraints ===
# Revolute joint at origin — hinge about world Z (XY-plane swing, Y-up)
# Rule 8: ChLinkLockRevolute uses local +Z as hinge axis; QUNIT => local +Z = world +Z (correct)
pivot = chrono.ChLinkLockRevolute()
pivot_pos = chrono.ChVector3d(0, 0, 0)
# The pivot point on the pendulum is at its near end: -L/2 from COM in body-local X
# Use body-local form to precisely locate the connection point on each body
pivot.Initialize(
    pendulum, ground, True,
    chrono.ChFramed(chrono.ChVector3d(-PENDULUM_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # near end of pendulum (body-local)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)                        # origin on ground (body-local)
)
sys.AddLink(pivot)

# === Visualization ===
# Full Irrlicht scene: window + Initialize() + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum — PyChrono 9.0")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up to match gravity convention
vis.Initialize()                                    # FIRST — Irrlicht rule
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 3), chrono.ChVector3d(0, -0.5, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.25, 0.25, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -1.2, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
