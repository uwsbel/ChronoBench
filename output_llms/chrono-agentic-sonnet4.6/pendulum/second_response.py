"""
Single pendulum simulation on the Moon using PyChrono (ChSystemNSC, Y-up).

Models a pendulum arm (cylinder, radius=0.1, height=1.5) attached to a fixed
ground body via a spherical joint. A sphere (radius=2) attached to the pendulum
at its pivot end visualizes the joint. Mass=2 kg, inertia=(0.4, 1.5, 1.5),
initial angular velocity applied, and lunar gravity (0, -1.62, 0). Expected
behavior: pendulum swings freely in 3D (spherical DOF) under moon gravity.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP  = 1e-3                                  # physics time step [s]
SIM_END    = 10.0                                  # simulation duration [s]
RENDER_FPS = 50.0                                  # render frame rate [Hz]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

PEND_MASS    = 2.0                                 # pendulum mass [kg]
PEND_INERTIA = chrono.ChVector3d(0.4, 1.5, 1.5)   # inertia tensor [kg·m²]
PEND_RADIUS  = 0.1                                 # cylinder radius [m]
PEND_HEIGHT  = 1.5                                 # cylinder height [m]
JOINT_SPHERE_RADIUS = 2.0                          # joint sphere visual radius [m]
INIT_ANG_VEL = chrono.ChVector3d(0.5, 0.0, 2.0)   # initial angular velocity [rad/s]
MOON_GRAVITY = chrono.ChVector3d(0, -1.62, 0)      # lunar gravitational acceleration [m/s²]

# Derived geometry
PIVOT_POS    = chrono.ChVector3d(0, 0, 0)          # pivot in world
PEND_COM_POS = chrono.ChVector3d(0, -PEND_HEIGHT / 2.0, 0)  # COM below pivot

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(MOON_GRAVITY)

# === Bodies ===
# Ground (fixed anchor for joint)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT_POS)
sys.AddBody(ground)

# Pendulum arm
pend = chrono.ChBody()
pend.SetMass(PEND_MASS)
pend.SetInertiaXX(PEND_INERTIA)
pend.SetPos(PEND_COM_POS)
pend.SetAngVelParent(INIT_ANG_VEL)  # initial angular velocity in world frame

# Cylinder visual aligned along Y-axis (body COM at center, top at +PEND_HEIGHT/2 local)
cyl_shape = chrono.ChVisualShapeCylinder(PEND_RADIUS, PEND_HEIGHT)
cyl_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
# Default cylinder axis is local Z; rotate to align with local Y (pendulum hangs along Y)
pend.AddVisualShape(cyl_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Joint sphere attached to pendulum at its top (pivot end) in pendulum local frame
joint_sphere = chrono.ChVisualShapeSphere(JOINT_SPHERE_RADIUS)
joint_sphere.SetColor(chrono.ChColor(0.9, 0.8, 0.1))
# Offset in pend local frame: top of pendulum is at (0, +PEND_HEIGHT/2, 0)
pend.AddVisualShape(joint_sphere, chrono.ChFramed(chrono.ChVector3d(0, PEND_HEIGHT / 2.0, 0)))

sys.AddBody(pend)

# === Joints / constraints ===
# Spherical joint connecting pendulum top to ground pivot
joint = chrono.ChLinkLockSpherical()
joint.Initialize(
    pend, ground, True,
    chrono.ChFramed(chrono.ChVector3d(0, PEND_HEIGHT / 2.0, 0)),  # top of pendulum in pend local
    chrono.ChFramed(PIVOT_POS)                                     # pivot in ground local
)
sys.AddLink(joint)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Moon Pendulum — Spherical Joint")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, 4, 8), chrono.ChVector3d(0, -1, 0))
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
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # main loop complete
