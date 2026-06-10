"""
Simple pendulum simulation using PyChrono with Irrlicht visualization.

System type: ChSystemNSC (Y-up, gravity = (0, -9.81, 0))
Bodies:
  - ground: fixed reference body (pivot support)
  - pend:   pendulum arm (box geometry, mass=1 kg)
Joint:
  - revolute hinge at ground pivot, hinge axis = world Z (XY swing plane)
Expected behavior:
  - Pendulum swings freely under gravity; angle and velocity oscillate
    indefinitely (no damping). Position and velocity logged periodically.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP  = 1e-3          # physics time step [s]
SIM_END    = 10.0          # simulation end time [s]
RENDER_FPS = 50.0          # render frame rate [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Pendulum geometry / physics
PEND_MASS   = 1.0          # pendulum mass [kg]
PEND_LENGTH = 1.0          # pendulum arm length [m]
PEND_WIDTH  = 0.05         # pendulum arm width/height [m]
# Inertia — simple direct assignment matching truth pattern
PEND_IX = 0.2
PEND_IY = 1.0
PEND_IZ = 1.0

# Pivot at world origin; pendulum COM at midpoint along -X from pivot
PIVOT_POS = chrono.ChVector3d(0, 0, 1)
PEND_POS  = chrono.ChVector3d(-PEND_LENGTH / 2.0, 0, 1)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Ground (fixed pivot body)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(PIVOT_POS)
ground_vis = chrono.ChVisualShapeSphere(0.04)
ground_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(ground_vis)
sys.Add(ground)

# Pendulum arm — manual ChBody for a link body
pend = chrono.ChBody()
pend.SetMass(PEND_MASS)
pend.SetInertiaXX(chrono.ChVector3d(PEND_IX, PEND_IY, PEND_IZ))
pend.SetPos(PEND_POS)
# Visual: cylinder along body-local X (Step 1: QUNIT since arm is along -X from pivot,
#         Step 2: QuatFromAngleY(CH_PI_2) rotates cyl default-Z to body-local X)
pend_cyl = chrono.ChVisualShapeCylinder(PEND_WIDTH / 2.0, PEND_LENGTH)
pend_cyl.SetColor(chrono.ChColor(0.7, 0.3, 0.1))
pend.AddVisualShape(pend_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.Add(pend)

# === Joints / constraints ===
# Revolute hinge at PIVOT_POS; hinge axis = world +Z (planar XY swing)
# ChLinkLockRevolute uses joint-frame local +Z as hinge axis; QUNIT => local +Z = world +Z
rev = chrono.ChLinkLockRevolute()
rev.Initialize(pend, ground, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
sys.AddLink(rev)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Simple Pendulum")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()

# === Main loop ===


try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()  # cache: fetched once per step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
