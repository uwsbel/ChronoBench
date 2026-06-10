"""
Slider-crank mechanism with modified joint types.
System: ChSystemNSC (Y-up, gravity -Y).
Bodies: floor (fixed), crank (driven by motor), connecting rod, piston.
Joints:
  - crank <-> floor: ChLinkMotorRotationSpeed (full motor-link, no separate revolute)
  - crank <-> rod:   ChLinkLockSpherical (ball-and-socket, per turn-3 modification)
  - rod   <-> piston: ChLinkLockSpherical (ball-and-socket, per turn-3 modification)
  - piston <-> floor: ChLinkLockPlanar (constrains piston to move/rotate in XY plane,
                                        replaces prismatic per turn-3 modification)
Expected behavior: crank rotates at constant speed; connecting rod transmits motion;
piston oscillates in the XY plane, constrained by the planar joint instead of
the original prismatic guide, so it can also rotate within the plane.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP   = 1e-3          # physics time step [s]
SIM_END     = 10.0          # simulation duration [s]
RENDER_FPS  = 50.0          # Irrlicht render rate [Hz]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

CRANK_LENGTH  = 0.5         # crank arm length [m]
ROD_LENGTH    = 1.5         # connecting rod length [m]
MOTOR_SPEED   = chrono.CH_PI  # crank angular speed [rad/s] (1 rev/2s)

# Body geometry
CRANK_MASS    = 1.0
CRANK_THICK   = 0.05
ROD_MASS      = 0.5
ROD_THICK     = 0.04
PISTON_MASS   = 1.0
PISTON_SIZE   = chrono.ChVector3d(0.2, 0.2, 0.15)  # full extents

# World-space positions (Y-up, mechanism in XY plane)
# Crank pivots at origin; crank pin at (+CRANK_LENGTH, 0, 0)
CRANK_PIVOT   = chrono.ChVector3d(0, 0, 0)
CRANK_PIN     = chrono.ChVector3d(CRANK_LENGTH, 0, 0)        # initial crank-pin
CRANK_CENTER  = chrono.ChVector3d(CRANK_LENGTH / 2, 0, 0)    # crank COM

# Rod goes from crank-pin to piston-pin; initial layout: rod along +X
PISTON_POS    = chrono.ChVector3d(CRANK_LENGTH + ROD_LENGTH, 0, 0)
ROD_CENTER    = chrono.ChVector3d(CRANK_LENGTH + ROD_LENGTH / 2, 0, 0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravityY()   # (0, -9.81, 0)
# Pure jointed MBS with no collision shapes — collision system intentionally omitted

# === Bodies ===

# Floor (fixed reference)
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor_vis = chrono.ChVisualShapeBox(4.0, 0.05, 0.3)
floor_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
floor.AddVisualShape(floor_vis, chrono.ChFramed(chrono.ChVector3d(CRANK_LENGTH, -0.05, 0), chrono.QUNIT))
sys.AddBody(floor)

# Crank (rotates about CRANK_PIVOT)
crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(chrono.ChVector3d(0.01, 0.1, 0.1))
crank.SetPos(CRANK_CENTER)
crank_vis = chrono.ChVisualShapeBox(CRANK_LENGTH, CRANK_THICK, CRANK_THICK)
crank_vis.SetColor(chrono.ChColor(0.8, 0.3, 0.1))
crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(crank)

# Connecting rod (links crank-pin to piston-pin)
rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.5, 0.5))
rod.SetPos(ROD_CENTER)
rod_vis = chrono.ChVisualShapeBox(ROD_LENGTH, ROD_THICK, ROD_THICK)
rod_vis.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(rod)

# Piston (slides and can rotate in XY plane)
piston = chrono.ChBody()
piston.SetMass(PISTON_MASS)
piston.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
piston.SetPos(PISTON_POS)
piston_vis = chrono.ChVisualShapeBox(PISTON_SIZE.x, PISTON_SIZE.y, PISTON_SIZE.z)
piston_vis.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
piston.AddVisualShape(piston_vis, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
sys.AddBody(piston)

# === Joints / constraints ===

# Crank <-> Floor: motor (full motor-link — no separate revolute)
# Hinge axis is world +Z; QUNIT maps local +Z to world +Z — correct for XY-plane rotation
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Crank <-> Rod: spherical (ball-and-socket) at crank-pin
# Body-local offsets: crank-pin is at (+CRANK_LENGTH/2, 0, 0) from crank COM;
#                     rod near-end is at (-ROD_LENGTH/2, 0, 0) from rod COM
joint_crank_rod = chrono.ChLinkLockSpherical()
joint_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(+CRANK_LENGTH / 2, 0, 0), chrono.QUNIT),  # on crank
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2, 0, 0), chrono.QUNIT),    # on rod
)
sys.AddLink(joint_crank_rod)

# Rod <-> Piston: spherical (ball-and-socket) at piston-pin
# Body-local offsets: rod far-end is at (+ROD_LENGTH/2, 0, 0) from rod COM;
#                     piston near-end is at (0, 0, 0) i.e. piston COM
joint_rod_piston = chrono.ChLinkLockSpherical()
joint_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(+ROD_LENGTH / 2, 0, 0), chrono.QUNIT),    # on rod
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),                  # on piston COM
)
sys.AddLink(joint_rod_piston)

# Piston <-> Floor: planar joint — constrains piston to move and rotate in XY plane
# ChLinkLockPlanar uses the joint frame's local Z as the normal to the plane.
# We want the piston in the XY plane (world Z=0), so normal = world +Z.
# QUNIT maps local +Z to world +Z — correct for XY plane constraint.
joint_piston_floor = chrono.ChLinkLockPlanar()
joint_piston_floor.Initialize(
    piston, floor,
    chrono.ChFramed(PISTON_POS, chrono.QUNIT),
)
sys.AddLink(joint_piston_floor)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank: Spherical + Planar Joints")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, 1.5, 3.5), chrono.ChVector3d(1.5, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===

frame = 0  # consecutive frame counter  # precomputed once

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
