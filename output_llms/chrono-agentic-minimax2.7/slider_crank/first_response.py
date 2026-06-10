"""
Slider-Crank Mechanism — PyChrono 9.0.0 MBS demo.

A simple crank-slider mechanism with:
- Fixed floor/truss ground body
- Crankshaft driven by a constant-speed motor at the origin
- Connecting rod linking crank-pin to wrist-pin
- Piston sliding on a fixed horizontal guide

Uses ChSystemNSC (non-smooth contact) with no collision shapes (pure joints only).
Visualized with Irrlicht.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
CRANK_CENTER_X = -1.0   # crank center world X [m]
CRANK_CENTER_Y = 0.5    # crank center world Y [m]
R = 0.4           # crank throw radius [m]
L = 1.5           # connecting rod length [m]
CRANK_THICK = 0.1  # crank cylinder thickness [m]
CRANKER_MOTOR_RPM = 60.0  # motor speed in revolutions per minute

time_step = 1e-3          # physics timestep [s]
sim_end = 10.0            # simulation duration [s]
render_fps = 50.0         # rendered frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity (Y-up: gravity = (0, -9.81, 0)) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# --- Ground / floor truss (fixed) ---
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# --- Crankshaft (flywheel) ---
# Cylinder along Y initially; Q_ROTATE_Y_TO_Z maps local Y -> world Z
# so the crank disc lies in the XY plane and rotates about world Z
crank_center = chrono.ChVector3d(CRANK_CENTER_X, CRANK_CENTER_Y, 0)
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, R, CRANK_THICK, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -CRANK_THICK / 2))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# --- Connecting rod ---
# Initial position: midpoint between crank-pin and wrist-pin
mrod = chrono.ChBodyEasyBox(L, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(R + L / 2, 0, 0))
sys.Add(mrod)

# --- Piston ---
# Initial position: at top-dead-center when crank is at theta=0
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(R + L, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # cylinder axis -> world X (horizontal sliding)
sys.Add(mpiston)

# === Joints ===
# Motor: crank <-> floor, spins crank about world Z at crank_center
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank, mfloor,
    chrono.ChFramed(crank_center)  # world-space frame at crank center
)
my_motor.SetSpeedFunction(chrono.ChFunctionConst(CRANKER_MOTOR_RPM * 2 * chrono.CH_PI / 60.0))  # rad/s
sys.Add(my_motor)

# Crank-rod revolute: 3-arg form — both frames are body-local
crank_pin_local = crank_center + chrono.ChVector3d(R, 0, 0)
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_pin_local))
sys.Add(mjointA)

# Rod-piston revolute: 3-arg form — both frames are body-local
wrist_pin_local = crank_center + chrono.ChVector3d(R + L, 0, 0)
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(wrist_pin_local))
sys.Add(mjointB)

# Piston-truss prismatic: slides along world X (Q_ROTATE_Z_TO_X maps frame Z -> world X)
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(
    mpiston, mfloor,
    chrono.ChFramed(wrist_pin_local, chrono.Q_ROTATE_Z_TO_X)
)
sys.Add(mjointC)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only recording scaffolding (stripped at ACCEPT) ===

# === Main loop ===
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break


# === Post-processing (review-only) ===
