"""
Single pendulum simulation on the Moon using PyChrono 9.0.0 with Irrlicht.

System type: ChSystemNSC (pure jointed MBS, no contact).
Bodies:
  - ground_body: fixed support frame with a sphere (r=2) visual at the pivot.
  - pendulum_body: rigid arm (cylinder, r=0.1, h=1.5) mass=2 kg,
    inertia=(0.4, 1.5, 1.5) kg·m², connected to ground via a spherical joint.

Moon gravity (0, -1.62, 0). The pendulum starts with an initial angular velocity
and swings freely under lunar gravity. Camera placed far away to see the full scene
including the large pivot sphere (r=2) and the pendulum arm below.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
# Geometry
PIVOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)   # pivot at origin (Y-up convention)
PEND_RADIUS   = 0.1      # cylinder radius [m] per prompt
PEND_HEIGHT   = 1.5      # cylinder height [m] per prompt
PEND_MASS     = 2.0      # pendulum mass [kg]
PEND_INERTIA  = chrono.ChVector3d(0.4, 1.5, 1.5)  # inertia tensor [kg·m²]
JOINT_SPHERE_R = 2.0     # sphere radius for joint visualization [m] per prompt
INIT_ANG_VEL  = 2.0      # initial angular velocity about world Z [rad/s] (inferred default — verify)

# Time
TIME_STEP  = 1e-3    # [s]
SIM_END    = 10.0    # [s]
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Derived positions: pendulum COM hangs half-length below pivot
PEND_COM_POS = chrono.ChVector3d(0.0, -PEND_HEIGHT / 2.0, 0.0)

# Camera: placed far back so the r=2 sphere and the hanging arm are both in view
CAM_EYE    = chrono.ChVector3d(12.0, 2.0, 0.0)   # far enough to see sphere+arm
CAM_TARGET = chrono.ChVector3d(0.0, -1.0, 0.0)   # look slightly below pivot

# === System & gravity ===  (Y-up, moon gravity)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -1.62, 0.0))
# Pure jointed MBS — no contact collision system needed

# === Bodies ===

# Ground body: fixed support with large sphere visual at pivot
ground_body = chrono.ChBody()
ground_body.SetFixed(True)
ground_body.SetPos(PIVOT_POS)
joint_sphere = chrono.ChVisualShapeSphere(JOINT_SPHERE_R)
joint_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
# Sphere is at origin in ground_body's local frame (same as PIVOT_POS in world)
ground_body.AddVisualShape(joint_sphere, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(ground_body)

# Pendulum body: rigid cylinder arm
pendulum_body = chrono.ChBody()
pendulum_body.SetMass(PEND_MASS)
pendulum_body.SetInertiaXX(PEND_INERTIA)
pendulum_body.SetPos(PEND_COM_POS)
pendulum_body.EnableCollision(False)
# Initial angular velocity about world Z
pendulum_body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, INIT_ANG_VEL))
# Cylinder visual: default axis body-local Z; rotate Z→Y with QuatFromAngleX(-PI/2)
cyl_shape = chrono.ChVisualShapeCylinder(PEND_RADIUS, PEND_HEIGHT)
cyl_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.9))
pendulum_body.AddVisualShape(
    cyl_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-chrono.CH_PI_2))
)
sys.AddBody(pendulum_body)

# === Joints / constraints ===  spherical joint (3-DOF rotation, replaces revolute)

# 5-arg form: rel_frames=True, local points on each body.
# pendulum_body COM at (0, -L/2, 0) world; top of rod in local frame = (0, +L/2, 0).
# ground_body at origin; pivot in its local frame = (0, 0, 0).
spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(
    pendulum_body, ground_body, True,
    chrono.ChFramed(chrono.ChVector3d(0.0, PEND_HEIGHT / 2.0, 0.0)),  # top of rod
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0)),                # pivot on ground
)
sys.AddLink(spherical_joint)

# === Visualization ===  full Irrlicht window (Y-up, wide FOV to see sphere+arm)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Moon Pendulum — Spherical Joint (r=2 pivot sphere)")
vis.Initialize()                     # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_EYE, CAM_TARGET)   # far camera to show full scene
vis.AddTypicalLights()
vis.AddGrid(
    2.0, 2.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -3.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

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
    pass  # scored-core placeholder so finally is syntactically valid when stripped
