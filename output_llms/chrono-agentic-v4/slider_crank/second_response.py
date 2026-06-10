"""
Slider-Crank Mechanism — turn 2
plan_type: mbs
System: ChSystemNSC (Non-Smooth Contact)
Objective: Collect piston position/speed vs crank angle data over 20 s, plot post-simulation.

Mechanism topology (ground-truth):
  crank  ↔ ground  : ChLinkMotorRotationSpeed  (drives the crank)
  crank  ↔ rod    : ChLinkLockRevolute        (crank-pin)
  rod    ↔ piston : ChLinkLockRevolute        (wrist-pin)
  piston ↔ ground : ChLinkLockPrismatic       (slides along X guide)
"""

import os
import math
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Named constants ===
CRANK_RADIUS = 0.5        # crank throw [m]
ROD_LENGTH   = 2.0         # connecting-rod length [m]
PISTON_RADIUS = 0.15       # piston radius [m]
CRANK_SPEED  = math.pi     # rad/s  (1 revolution per 2 s)
SIM_DURATION = 20.0        # simulation duration [s]
TIME_STEP    = 1e-3        # physics time step [s]
RENDER_FPS   = 50.0        # render frames per second


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Collision system required — piston slides on fixed ground guide (contact present)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

solver_max_iter = 500
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(solver_max_iter)


# === Ground (fixed floor/truss) ===
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, -CRANK_RADIUS, 0.0))
ground.AddVisualShape(chrono.ChVisualShapeBox(6.0, 0.1, 1.0))
ground.EnableCollision(True)
sys.AddBody(ground)


# === Crank body ===
# Cylinder axis = body-local Y; rotates in XY plane about Z
crank = chrono.ChBody()
crank_mass = 1.0
crank.SetMass(crank_mass)
crank.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
crank.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
crank.SetRot(chrono.QUNIT)
sys.AddBody(crank)

crank_vis = chrono.ChVisualShapeCylinder(CRANK_RADIUS, 0.05)
crank_vis.SetColor(chrono.ChColor(0.7, 0.2, 0.2))
crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Crank-pin visual (small sphere at crank tip)
pin_vis = chrono.ChVisualShapeSphere(0.05)
crank.AddVisualShape(pin_vis, chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0), chrono.QUNIT))


# === Connecting rod ===
# COM at midpoint between crank-pin and wrist-pin
rod_com_x = CRANK_RADIUS + ROD_LENGTH / 2.0 * math.cos(0.0)  # initial horizontal
rod_com_y = 0.0 + ROD_LENGTH / 2.0 * math.sin(0.0)
rod = chrono.ChBody()
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
rod.SetPos(chrono.ChVector3d(rod_com_x, rod_com_y, 0.0))
sys.AddBody(rod)

rod_vis = chrono.ChVisualShapeCylinder(0.04, ROD_LENGTH)
rod_vis.SetColor(chrono.ChColor(0.3, 0.6, 0.3))
rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))


# === Piston (slider) ===
# Piston slides along X-axis; initial x = crank radius + rod length
piston_init_x = CRANK_RADIUS + ROD_LENGTH
piston = chrono.ChBody()
piston.SetMass(0.5)
piston.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
piston.SetPos(chrono.ChVector3d(piston_init_x, 0.0, 0.0))
sys.AddBody(piston)

piston_vis = chrono.ChVisualShapeCylinder(PISTON_RADIUS, 0.3)
piston_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
piston.AddVisualShape(piston_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))


# === Joints ===
# Crank ↔ ground  (motor — full motor-link, no separate revolute needed)
motor_frame = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT)
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crank, ground, motor_frame)
crank_motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(crank_motor)

# Crank ↔ rod  (revolute at crank-pin)
crank_pin_pos = chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0)
crank_pin_frame = chrono.ChFramed(crank_pin_pos, chrono.QUNIT)
crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crank, rod, crank_pin_frame)
sys.AddLink(crank_rod_joint)

# Rod ↔ piston  (revolute at wrist-pin)
wrist_pin_offset = chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0)  # rod COM to wrist pin
rod_wrist_frame = chrono.ChFramed(wrist_pin_offset, chrono.QUNIT)
rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(rod, piston, rod_wrist_frame)
sys.AddLink(rod_piston_joint)

# Piston ↔ ground  (prismatic along X guide)
prismatic_frame = chrono.ChFramed(chrono.ChVector3d(piston_init_x, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_X)
piston_ground_joint = chrono.ChLinkLockPrismatic()
piston_ground_joint.Initialize(piston, ground, prismatic_frame)
sys.AddLink(piston_ground_joint)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank — turn 2")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -1.5, 3.0), chrono.ChVector3d(1.5, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))


# === Precomputed constants ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))


# === Data arrays for plotting ===
array_time  = []
array_angle = []
array_pos   = []
array_speed = []


# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < SIM_DURATION:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        # Collect data at every physics step
        array_time.append(sys.GetChTime())
        array_angle.append(crank_motor.GetMotorAngle())
        array_pos.append(piston.GetPos().x)
        array_speed.append(piston.GetPosDt().x)

        sys.DoStepDynamics(TIME_STEP)
        if sys.GetChTime() >= SIM_DURATION:
            break


# === Review-only: post-simulation matplotlib plots ===
