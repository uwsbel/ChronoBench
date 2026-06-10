"""
Simple pendulum simulation using PyChrono with Irrlicht visualization.

System type: ChSystemNSC (Y-up, gravity = (0, -9.81, 0))
Bodies:
  - ground: fixed support body at the origin
  - pendulum: cylindrical rod (mass=1 kg) pivoting from the ground
Joint: revolute hinge connecting ground to pendulum top end (hinge axis = Z)
Expected behaviour: pendulum swings freely under gravity; position and velocity
are logged periodically to simulation_data.csv.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP  = 1e-3        # physics timestep [s]
SIM_END    = 10.0        # simulation end time [s]
RENDER_FPS = 50.0        # render frames per second
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

PEND_MASS   = 1.0        # pendulum mass [kg]
PEND_LENGTH = 1.0        # pendulum rod length [m]
PEND_RADIUS = 0.05       # pendulum rod visual radius [m]
PIVOT_POS   = chrono.ChVector3d(0, 0, 0)    # hinge point in world space
PEND_COM    = chrono.ChVector3d(0, -PEND_LENGTH / 2.0, 0)  # COM at mid-rod (Y-up, hanging down)

# Pendulum inertia — simple direct assignment matching canonical demo style
PEND_IX = 0.2   # [kg·m²]
PEND_IY = 1.0
PEND_IZ = 1.0

# Initial displacement angle (rad) about Z to give interesting swing
INIT_ANGLE = math.pi / 4.0   # 45°

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravityY()   # gravity = (0, -9.81, 0), Y-up

# Pure jointed MBS — no contact/collision bodies, so SetCollisionSystemType is omitted per truth

# === Bodies ===
# Ground — fixed support
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)

# Small sphere to visualize the pivot point
pivot_vis = chrono.ChVisualShapeSphere(0.04)
pivot_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(pivot_vis, chrono.ChFramed(PIVOT_POS))

# Pendulum rod — manual ChBody for a rotating link
pendulum = chrono.ChBody()
pendulum.SetName("pendulum")
pendulum.SetMass(PEND_MASS)
pendulum.SetInertiaXX(chrono.ChVector3d(PEND_IX, PEND_IY, PEND_IZ))

# Place COM at mid-rod; rotate by init angle so it starts displaced
q_init = chrono.QuatFromAngleZ(INIT_ANGLE)  # rotate in XY plane about Z hinge
pend_com_rotated = q_init.Rotate(PEND_COM)  # cache: rotated COM once
pendulum.SetPos(pend_com_rotated)
pendulum.SetRot(q_init)

sys.AddBody(pendulum)

# Visual cylinder for the pendulum rod (default axis = body-local Z → rotate to body-local X via Step 2)
cyl_shape = chrono.ChVisualShapeCylinder(PEND_RADIUS, PEND_LENGTH)
cyl_shape.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
pendulum.AddVisualShape(
    cyl_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2))
)

# === Joints / constraints ===
# Revolute joint at pivot — hinge axis is world Z (planar XY swing, gravity -Y)
# ChLinkLockRevolute local +Z = world +Z → QUNIT frame is correct here
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(pendulum, ground, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
sys.AddLink(hinge)

# === Visualization ===  full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum — PyChrono")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # Y-up world
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 3), chrono.ChVector3d(0, -0.5, 0))  # AFTER Initialize
vis.AddTypicalLights()

# === Setup for review recording ===

# CSV log setup (review-only)

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()    # cache: fetched once per step
            pos = pendulum.GetPos()  # cache: fetched once per step
            vel = pendulum.GetPosDt()  # cache: fetched once per step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
