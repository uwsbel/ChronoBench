"""
Single pendulum simulation on the moon using PyChrono (NSC, Y-up).

Models a rigid pendulum arm suspended from a fixed ground via a spherical joint.
The pendulum has mass 2 kg and inertia tensor (0.4, 1.5, 1.5) kg·m².
The pivot joint is visualized as a sphere of radius 2.
The pendulum arm is visualized as a cylinder of radius 0.1 and height 1.5.
Gravity is lunar: (0, -1.62, 0) m/s².
The pendulum is given an initial angular velocity at the start.
System: ChSystemNSC (pure jointed MBS — no contact, no collision shapes).
Expected: pendulum swings under reduced lunar gravity with initial spin.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP  = 1e-3          # physics time step [s]
SIM_END    = 10.0          # simulation duration [s]
RENDER_FPS = 50.0          # render frame rate [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Pendulum geometry / physics
PEND_MASS     = 2.0                          # pendulum mass [kg]
PEND_INERTIA  = chrono.ChVector3d(0.4, 1.5, 1.5)  # inertia tensor [kg·m²]
CYL_RADIUS    = 0.1                          # cylinder visual radius [m]
CYL_HEIGHT    = 1.5                          # cylinder visual height [m]
SPHERE_RADIUS = 2.0                          # joint sphere visual radius [m]
INIT_ANG_VEL  = chrono.ChVector3d(0, 0, 2.0)  # initial angular velocity [rad/s]

# Pivot and pendulum COM positions
PIVOT_POS   = chrono.ChVector3d(0, 0, 0)
# Pendulum arm: pivot at top, COM halfway down → COM at (0, -CYL_HEIGHT/2, 0)
PEND_POS    = chrono.ChVector3d(0, -CYL_HEIGHT / 2.0, 0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
# Lunar gravitational acceleration
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# === Bodies ===
# Ground (fixed pivot support)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT_POS)

# Sphere visual at joint to mark pivot point
sphere_shape = chrono.ChVisualShapeSphere(SPHERE_RADIUS)
sphere_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(sphere_shape, chrono.ChFramed(PIVOT_POS))
sys.AddBody(ground)

# Pendulum arm body
pend = chrono.ChBody()
pend.SetMass(PEND_MASS)
pend.SetInertiaXX(PEND_INERTIA)
pend.SetPos(PEND_POS)
# Initial angular velocity in world frame
pend.SetAngVelParent(INIT_ANG_VEL)

# Cylinder visual for the pendulum arm (default axis Z → offset to point along Y)
cyl_shape = chrono.ChVisualShapeCylinder(CYL_RADIUS, CYL_HEIGHT)
cyl_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
# Cylinder default axis is body-local Z; rotate by 90° about X so it aligns with body Y
pend.AddVisualShape(
    cyl_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2))
)
sys.AddBody(pend)

# === Joints / constraints ===
# Spherical joint at pivot: allows all 3 rotations, no translations
sph_joint = chrono.ChLinkLockSpherical()
sph_joint.Initialize(
    pend, ground,
    True,
    chrono.ChFramed(chrono.ChVector3d(0,  CYL_HEIGHT / 2.0, 0)),  # top of arm (pend local)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0))                    # origin of ground
)
sys.AddLink(sph_joint)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Single Pendulum — Moon Gravity — Spherical Joint")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -2, 8), chrono.ChVector3d(0, -1, 0))
vis.AddTypicalLights()

# === Main loop ===

# CSV logging setup (review-only block)

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
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
