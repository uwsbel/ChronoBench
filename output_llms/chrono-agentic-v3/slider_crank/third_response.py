"""
Slider-crank mechanism simulation using PyChrono (ChSystemNSC, Y-up).

Models a motor-driven slider-crank with the following joint topology:
  - Crank  ↔ floor : ChLinkMotorRotationSpeed (full motor-link, no extra revolute)
  - Crank  ↔ rod   : ChLinkLockSpherical (spherical/ball-and-socket joint)
  - Rod    ↔ piston: ChLinkLockSpherical (spherical/ball-and-socket joint)
  - Piston ↔ floor : ChLinkLockPlanar   (planar joint — constrains piston to XY plane)

The motor spins the crank at constant angular speed; the connecting rod converts
crank rotation into planar oscillation of the piston, constrained to the XY plane.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP   = 1e-3       # physics step size [s]
SIM_END     = 10.0       # simulation end time [s]
RENDER_FPS  = 50.0       # render frames per second
CRANK_SPEED = math.pi    # crank angular speed [rad/s] — one half-turn per second

# Geometry
CRANK_LENGTH = 0.5   # crank arm length [m]
ROD_LENGTH   = 1.5   # connecting rod length [m]
CRANK_MASS   = 1.0   # crank mass [kg]
ROD_MASS     = 1.0   # connecting rod mass [kg]
PISTON_MASS  = 1.0   # piston mass [kg]

# Inertia (simple box-like assignment — matches slider_crank truth pattern)
CRANK_INERTIA  = chrono.ChVector3d(0.05, 0.1, 0.1)
ROD_INERTIA    = chrono.ChVector3d(0.05, 0.5, 0.5)
PISTON_INERTIA = chrono.ChVector3d(0.05, 0.05, 0.05)

# precomputed once — render batch size
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# ChSystemNSC — pure jointed MBS, no contact (no SetCollisionSystemType needed)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up

# === Bodies ===

# Fixed floor / ground
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor_vis = chrono.ChVisualShapeBox(6.0, 0.1, 0.4)
floor_vis.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.AddVisualShape(floor_vis, chrono.ChFramed(chrono.ChVector3d(0, -0.05, 0)))
sys.AddBody(floor)

# Crank: rotates about crank pivot at (0, 0, 0); its COM is at half-length
crank_pivot = chrono.ChVector3d(0.0, 0.0, 0.0)
crank_com   = chrono.ChVector3d(CRANK_LENGTH / 2.0, 0.0, 0.0)
crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(CRANK_INERTIA)
crank.SetPos(crank_com)
crank.SetRot(chrono.QUNIT)
crank_vis = chrono.ChVisualShapeCylinder(0.05, CRANK_LENGTH)
crank_vis.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
crank.AddVisualShape(crank_vis,
                     chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

# Crank-pin position (far end of crank)
crank_pin = chrono.ChVector3d(CRANK_LENGTH, 0.0, 0.0)

# Connecting rod: spans from crank-pin to piston-pin
# Initial configuration: crank at angle 0 (horizontal), rod extends to the right
piston_pin_x = CRANK_LENGTH + ROD_LENGTH
rod_com = chrono.ChVector3d((crank_pin.x + piston_pin_x) / 2.0, 0.0, 0.0)
rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(ROD_INERTIA)
rod.SetPos(rod_com)
rod.SetRot(chrono.QUNIT)
rod_vis = chrono.ChVisualShapeCylinder(0.04, ROD_LENGTH)
rod_vis.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
rod.AddVisualShape(rod_vis,
                   chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

# Piston: slides in XY plane, starts at (crank_pin_x + rod_length, 0, 0)
piston_pos = chrono.ChVector3d(piston_pin_x, 0.0, 0.0)
piston = chrono.ChBody()
piston.SetMass(PISTON_MASS)
piston.SetInertiaXX(PISTON_INERTIA)
piston.SetPos(piston_pos)
piston.SetRot(chrono.QUNIT)
piston_vis = chrono.ChVisualShapeBox(0.2, 0.2, 0.3)
piston_vis.SetColor(chrono.ChColor(0.4, 0.8, 0.2))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints ===

# 1. Crank ↔ floor: motor (full motor-link — no separate revolute)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    crank, floor,
    chrono.ChFramed(crank_pivot, chrono.QUNIT)
)
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

# 2. Crank ↔ rod: spherical (ball-and-socket) at crank-pin
#    5-arg form: body1=crank, body2=rod, rel_frames=True,
#    frame on crank in crank's local coords = +CRANK_LENGTH/2 from COM
joint_crank_rod = chrono.ChLinkLockSpherical()
joint_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),  # crank local
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT)    # rod local
)
sys.AddLink(joint_crank_rod)

# 3. Rod ↔ piston: spherical (ball-and-socket) at piston-pin
joint_rod_piston = chrono.ChLinkLockSpherical()
joint_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),   # rod local
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT)                 # piston local (COM)
)
sys.AddLink(joint_rod_piston)

# 4. Piston ↔ floor: planar joint — constrains piston to the XY plane
#    ChLinkLockPlanar: the joint frame's local XY is the allowed motion plane;
#    frame Z is the normal (constrained direction). With QUNIT the XY plane of
#    the world IS the constraint plane, which is the x-y plane here.
joint_piston_floor = chrono.ChLinkLockPlanar()
joint_piston_floor.Initialize(
    piston, floor,
    chrono.ChFramed(piston_pos, chrono.QUNIT)
)
sys.AddLink(joint_piston_floor)

# === Visualization ===
# Full Irrlicht block: Initialize first, then sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank: Spherical Joints + Planar Piston Constraint")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0, 2.0, 3.0), chrono.ChVector3d(1.5, 0.0, 0.0))
vis.AddTypicalLights()

# === Output directory setup (review-only dirs) ===

# === Main loop ===
frame = 0  # consecutive frame counter for Irrlicht capture

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad parameter
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
