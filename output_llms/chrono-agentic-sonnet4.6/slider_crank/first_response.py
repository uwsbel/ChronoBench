"""
Slider-Crank Mechanism Simulation.

Models a classic crank-slider (Scotch-yoke) mechanism using PyChrono with
ChSystemNSC. The system comprises:
  - A fixed floor (truss) body
  - A crankshaft driven at constant angular speed by a ChLinkMotorRotationSpeed
  - A connecting rod linked to the crank pin via a revolute joint
  - A piston linked to the rod via a revolute joint and guided by a prismatic
    joint to the floor

The motor provides constant angular velocity, converting rotation to linear
sliding motion. Visualized using the Irrlicht renderer with sky, lights, grid,
textures, and logo.
"""

import math
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Geometry / physics parameters
CRANK_LENGTH    = 0.5          # crank arm half-length [m] (crank pin offset from pivot)
ROD_LENGTH      = 1.2          # connecting rod length [m]
CRANK_RADIUS    = 0.04         # cylinder visual radius for crank [m]
ROD_RADIUS      = 0.03         # cylinder visual radius for connecting rod [m]
PISTON_SIZE     = 0.15         # piston box half-size [m] (cube)
FLOOR_HALF_X    = 3.0          # floor half-extents [m]
FLOOR_HALF_Z    = 0.05         # floor thickness half-extent [m]
CRANK_MASS      = 0.5          # crank body mass [kg]
ROD_MASS        = 0.3          # connecting rod mass [kg]
PISTON_MASS     = 0.2          # piston mass [kg]
MOTOR_SPEED     = chrono.CH_PI  # constant angular speed [rad/s] (half-turn/s)

# World positions derived from geometry
# Crank pivot is at world origin at height = FLOOR_HALF_Z (on top of floor)
PIVOT_X         = 0.0
PIVOT_Y         = 0.0
PIVOT_Z         = FLOOR_HALF_Z                  # floor top surface

# Crank COM is offset CRANK_LENGTH along X from pivot at Z=PIVOT_Z
CRANK_COM_X     = CRANK_LENGTH                   # crank starts horizontal
CRANK_COM_Z     = PIVOT_Z

# Crank pin (far end of crank) is at 2×CRANK_LENGTH from pivot
CRANKPIN_X      = 2.0 * CRANK_LENGTH
CRANKPIN_Z      = PIVOT_Z

# Rod COM is at midpoint between crank pin and piston wrist pin
# Rod lies horizontal at start: piston wrist pin at CRANKPIN_X + ROD_LENGTH
WRISTPIN_X      = CRANKPIN_X + ROD_LENGTH
WRISTPIN_Z      = PIVOT_Z
ROD_COM_X       = (CRANKPIN_X + WRISTPIN_X) / 2.0
ROD_COM_Z       = PIVOT_Z

# Piston COM at wrist-pin position (piston centered on wrist pin)
PISTON_COM_X    = WRISTPIN_X
PISTON_COM_Z    = PIVOT_Z

# Simulation time settings
time_step       = 1e-3          # physics step [s]
sim_end         = 10.0          # simulation end [s]
render_fps      = 50.0          # Irrlicht render target [fps]
render_every    = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# NSC (non-smooth contact) system — pure jointed MBS with no contact geometry
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# No collision system needed — this is a pure joint mechanism with no contact

# === Bodies ===

# --- Floor (truss) — fixed, acts as ground anchor ---
floor = chrono.ChBody()
floor.SetFixed(True)
floor_box = chrono.ChVisualShapeBox(2.0 * FLOOR_HALF_X, 2.0 * FLOOR_HALF_X, 2.0 * FLOOR_HALF_Z)
floor_box.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
# Floor COM at z=0; its top surface is at FLOOR_HALF_Z
floor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
floor.AddVisualShape(floor_box)
sys.AddBody(floor)

# --- Crankshaft — rotates about Z-axis at the pivot ---
# Starts horizontal: COM midway between pivot and crank pin along X
crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
crank.SetPos(chrono.ChVector3d(CRANK_COM_X, 0.0, CRANK_COM_Z))
# Crank lies along X — SetRot to identity (body-local X = world X)
crank.SetRot(chrono.QUNIT)
crank_cyl = chrono.ChVisualShapeCylinder(CRANK_RADIUS, CRANK_LENGTH)
crank_cyl.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
# Step 2 visual formula: rotate default Z-axis cylinder to point along body X
crank.AddVisualShape(crank_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

# --- Connecting rod — links crank pin to piston wrist pin ---
rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(chrono.ChVector3d(0.03, 0.03, 0.03))
rod.SetPos(chrono.ChVector3d(ROD_COM_X, 0.0, ROD_COM_Z))
rod.SetRot(chrono.QUNIT)
rod_cyl = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LENGTH)
rod_cyl.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

# --- Piston — translates along X, guided by prismatic joint to floor ---
piston = chrono.ChBody()
piston.SetMass(PISTON_MASS)
piston.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
piston.SetPos(chrono.ChVector3d(PISTON_COM_X, 0.0, PISTON_COM_Z))
piston_box = chrono.ChVisualShapeBox(PISTON_SIZE, PISTON_SIZE, PISTON_SIZE)
piston_box.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
piston.AddVisualShape(piston_box)
sys.AddBody(piston)

# === Joints / constraints ===
# Hinge axis is world +Z (normal to XZ motion plane, perpendicular to Z).
# Since bodies move in XZ plane (gravity along -Z, mechanism in XZ plane),
# the rotation axis is world +Y. Hinge frame: local +Z must align with world +Y.
q_hinge_y = chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)

# --- Motor: crank ↔ floor — ChLinkMotorRotationSpeed is a full motor-link ---
# (no separate revolute needed at the same pivot)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    crank, floor,
    chrono.ChFramed(chrono.ChVector3d(PIVOT_X, 0.0, PIVOT_Z), q_hinge_y)
)
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# --- Crank-rod revolute — crank pin between crank and rod ---
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    crank, rod,
    chrono.ChFramed(chrono.ChVector3d(CRANKPIN_X, 0.0, CRANKPIN_Z), q_hinge_y)
)
sys.AddLink(joint_crank_rod)

# --- Rod-piston revolute — wrist pin between rod and piston ---
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    rod, piston,
    chrono.ChFramed(chrono.ChVector3d(WRISTPIN_X, 0.0, WRISTPIN_Z), q_hinge_y)
)
sys.AddLink(joint_rod_piston)

# --- Piston-floor prismatic — piston slides along X on the fixed guide ---
# Prismatic frame: local +Z must align with the sliding axis (world +X).
joint_piston_floor = chrono.ChLinkLockPrismatic()
joint_piston_floor.Initialize(
    piston, floor,
    chrono.ChFramed(chrono.ChVector3d(PISTON_COM_X, 0.0, PISTON_COM_Z), chrono.Q_ROTATE_Z_TO_X)
)
sys.AddLink(joint_piston_floor)

# === Visualization ===
# Full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()  # Initialize FIRST (Irrlicht: opposite call order to VSG)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0, -4.0, 2.0), chrono.ChVector3d(2.0, 0.0, 0.5))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 30, 30,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, FLOOR_HALF_Z), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad constraint
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
