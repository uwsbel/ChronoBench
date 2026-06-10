"""
Slider-Crank Mechanism with Spherical Joints and Planar Constraint.

Models a slider-crank mechanism where:
  - The crank (length 0.2 m) is driven at constant angular speed by a motor linked
    to the fixed floor/truss.
  - The connecting rod (length 0.5 m) joins the crank-pin to the piston-pin using
    SPHERICAL (ball-and-socket) joints at both ends.
  - The piston is constrained to the XY plane via a PLANAR joint against the floor,
    replacing the classical prismatic guide. The planar joint allows translation in X
    and Y and rotation about Z, while preventing out-of-plane (Z) translation and X/Y
    rotations.

System type : ChSystemNSC (rigid multi-body, no contact — pure jointed MBS)
Bodies       : floor (fixed), crank, connecting rod, piston
Motor        : ChLinkMotorRotationSpeed (full motor-link at crank pivot)
Joints       : ChLinkLockSpherical (crank-pin and wrist-pin),
               ChLinkLockPlanar (piston to floor)
Expected     : crank rotates continuously; rod oscillates; piston translates/rotates
               in the XY plane (near-horizontal when the planar joint is the guide).
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Mechanism geometry
CRANK_LEN   = 0.2   # crank arm half-length (pivot to crank-pin) [m]
ROD_LEN     = 0.5   # connecting rod length [m]
MOTOR_RPM   = 60.0  # crank rotation speed [rpm]
MOTOR_RAD_S = MOTOR_RPM * 2.0 * math.pi / 60.0  # precomputed once

# Body properties
CRANK_MASS  = 1.0   # [kg]
ROD_MASS    = 0.5   # [kg]
PISTON_MASS = 2.0   # [kg]
DENSITY_STEEL = 7800.0  # [kg/m³] — for easy factory calls

# Simulation timing
TIME_STEP   = 1e-3        # [s]
SIM_END     = 5.0         # [s]
RENDER_FPS  = 50.0
# precomputed once: physics steps between rendered frames
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# === System & gravity ===
sys = chrono.ChSystemNSC()
# Y-up world convention (standard MBS demos); gravity along -Y
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# Pure jointed MBS — no collision shapes → do NOT call SetCollisionSystemType

# === Bodies ===

# -- Floor (fixed truss) --
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetMass(1.0)
floor.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor_vis = chrono.ChVisualShapeBox(1.0, 0.05, 0.3)
floor_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
floor.AddVisualShape(floor_vis, chrono.ChFramed(chrono.ChVector3d(0.3, -0.05, 0)))
sys.AddBody(floor)

# -- Crank --
# Pivot at origin; crank-pin at (CRANK_LEN, 0, 0) in world initially.
# Model as a cylinder along X (body-local X = crank direction).
crank_pin_world = chrono.ChVector3d(CRANK_LEN, 0, 0)
crank_mid_world = chrono.ChVector3d(CRANK_LEN / 2.0, 0, 0)  # body COM at midpoint

crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
# Thin rod inertia about short axes: I = (1/12)*m*L^2; about long axis: small
Ic_long = 0.5 * CRANK_MASS * (0.02 ** 2)          # ~thin rod about its own axis
Ic_trans = (1.0 / 12.0) * CRANK_MASS * CRANK_LEN ** 2
crank.SetInertiaXX(chrono.ChVector3d(Ic_long, Ic_trans, Ic_trans))
crank.SetPos(crank_mid_world)
crank.SetRot(chrono.QUNIT)  # body-local X along world X
crank.EnableCollision(False)
sys.AddBody(crank)

crank_cyl = chrono.ChVisualShapeCylinder(0.02, CRANK_LEN)
crank.AddVisualShape(crank_cyl,
                     chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
crank_pin_vis = chrono.ChVisualShapeSphere(0.03)
crank_pin_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_pin_vis,
                     chrono.ChFramed(chrono.ChVector3d(CRANK_LEN / 2.0, 0, 0)))

# -- Connecting rod --
# Initially from crank-pin to piston-pin; rod lies along X.
# Piston-pin world position: crank-pin.x + ROD_LEN  (rod starts horizontal)
piston_pin_world = chrono.ChVector3d(CRANK_LEN + ROD_LEN, 0, 0)
rod_mid_world    = chrono.ChVector3d(CRANK_LEN + ROD_LEN / 2.0, 0, 0)

rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
Ir_long  = 0.5 * ROD_MASS * (0.015 ** 2)
Ir_trans = (1.0 / 12.0) * ROD_MASS * ROD_LEN ** 2
rod.SetInertiaXX(chrono.ChVector3d(Ir_long, Ir_trans, Ir_trans))
rod.SetPos(rod_mid_world)
rod.SetRot(chrono.QUNIT)
rod.EnableCollision(False)
sys.AddBody(rod)

rod_cyl = chrono.ChVisualShapeCylinder(0.015, ROD_LEN)
rod.AddVisualShape(rod_cyl,
                   chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# -- Piston --
piston = chrono.ChBody()
piston.SetMass(PISTON_MASS)
Ip = (1.0 / 12.0) * PISTON_MASS * (0.08 ** 2 + 0.08 ** 2)
piston.SetInertiaXX(chrono.ChVector3d(Ip, Ip, Ip))
piston.SetPos(piston_pin_world)   # COM at the wrist-pin initially
piston.SetRot(chrono.QUNIT)
piston.EnableCollision(False)
sys.AddBody(piston)

piston_box = chrono.ChVisualShapeBox(0.1, 0.08, 0.08)
piston_box.SetColor(chrono.ChColor(0.2, 0.5, 0.9))
piston.AddVisualShape(piston_box)

# === Joints / constraints ===

# -- Motor: crank ↔ floor (full motor-link — no separate revolute) --
# ChLinkMotorRotationSpeed is a FULL motor-link; it already imposes the revolute
# constraint at the pivot plus prescribes the angular speed.
motor = chrono.ChLinkMotorRotationSpeed()
pivot_frame = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),   # pivot at world origin
    chrono.QUNIT                  # hinge axis = world Z (local +Z of frame)
)
motor.Initialize(crank, floor, pivot_frame)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_RAD_S))
sys.AddLink(motor)

# -- Spherical joint: crank-pin (crank ↔ rod) --
# Ball-and-socket: no rotational DOF removed, only translation locked.
# Connection point in crank's local frame: +CRANK_LEN/2 along local X
# Connection point in rod's   local frame: -ROD_LEN/2  along local X
sph_crank_rod = chrono.ChLinkLockSpherical()
sph_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_LEN / 2.0, 0, 0)),   # on crank (local)
    chrono.ChFramed(chrono.ChVector3d(-ROD_LEN / 2.0, 0, 0))      # on rod   (local)
)
sys.AddLink(sph_crank_rod)

# -- Spherical joint: wrist-pin (rod ↔ piston) --
# Connection point in rod's    local frame: +ROD_LEN/2  along local X
# Connection point in piston's local frame: origin (COM = wrist-pin initially)
sph_rod_piston = chrono.ChLinkLockSpherical()
sph_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LEN / 2.0, 0, 0)),   # on rod    (local)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0))                 # on piston (local)
)
sys.AddLink(sph_rod_piston)

# -- Planar joint: piston ↔ floor (XY-plane constraint) --
# ChLinkLockPlanar constrains the piston to remain in the plane defined by the
# joint frame. The frame's local Z is the plane normal; we set it to world Z so
# the piston is constrained to the XY plane. This blocks Z-translation and
# X/Y-rotation, while allowing X/Y translation and Z-rotation.
planar = chrono.ChLinkLockPlanar()
planar_frame = chrono.ChFramed(
    piston_pin_world,   # frame origin at initial piston position
    chrono.QUNIT        # local Z = world Z → plane normal is world Z → XY plane
)
planar.Initialize(piston, floor, planar_frame)
sys.AddLink(planar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank: Spherical Joints + Planar Constraint")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up scene
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.3, 1.5), chrono.ChVector3d(0.4, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1, 0.1, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -0.02, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Main loop ===

# CSV logging setup (review-only block)

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()  # cache: fetched once per physics step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
