"""
Slider-Crank Mechanism Simulation (PyChrono 9.0.x, Irrlicht)
=============================================================
Models a classic slider-crank mechanism in the XY plane with Y-up gravity.
Four rigid bodies:
  - floor/truss: fixed ground reference with concrete texture
  - crankshaft: rotates about origin, driven at constant angular speed by a motor
  - connecting rod: links the crank pin to the piston wrist pin
  - piston: slides horizontally (X direction) on the fixed guide rail

System type : ChSystemNSC (rigid, impulsive; pure jointed MBS — no collision shapes)
Expected behaviour: crank spins at constant angular speed; piston oscillates
  sinusoidally along X driven through the connecting rod.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
CRANK_SPEED     = math.pi          # rad/s (~30 rpm), constant motor speed
CRANK_LENGTH    = 0.5              # m — crank arm (pivot to crank pin)
ROD_LENGTH      = 1.0              # m — connecting rod length
CRANK_DENSITY   = 7800.0           # kg/m³ (steel)
ROD_DENSITY     = 2700.0           # kg/m³ (aluminium)
PISTON_DENSITY  = 4000.0           # kg/m³
FLOOR_DENSITY   = 1000.0           # kg/m³ (visual only, fixed)
TIME_STEP       = 1e-3             # s — physics timestep (1 ms, high-precision MBS)
SIM_END         = 10.0             # s — simulation duration
RENDER_FPS      = 50.0             # target review-video frame rate
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Mechanism layout: crank pivot at world origin; mechanism in XY plane
CRANK_PIVOT     = chrono.ChVector3d(0.0, 0.0, 0.0)
# Crank pin starts at angle=0 (rightmost position)
CRANK_PIN_INIT  = chrono.ChVector3d(CRANK_LENGTH, 0.0, 0.0)
# Piston fully extended at t=0
PISTON_INIT_X   = CRANK_LENGTH + ROD_LENGTH          # precomputed once
PISTON_INIT_POS = chrono.ChVector3d(PISTON_INIT_X, 0.0, 0.0)
# Connecting rod COM at midpoint
ROD_MID_POS     = chrono.ChVector3d(0.5 * (CRANK_LENGTH + PISTON_INIT_X), 0.0, 0.0)

# === System & gravity ===
# ChSystemNSC: rigid/impulsive — correct for pure jointed MBS with no contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))  # Y-up
# No collision system: pure jointed MBS, no collision shapes used

# === Bodies ===
# --- Floor / truss (fixed ground reference) ---
floor = chrono.ChBodyEasyBox(6.0, 0.1, 0.4, FLOOR_DENSITY, True, False)
floor.SetPos(chrono.ChVector3d(1.5, -0.12, 0.0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# --- Crankshaft ---
# Modeled as a thin disk (cylinder axis = Z) at the pivot plus an arm visual
crank_radius = 0.05              # m
crank_width  = 0.05              # m half-thickness
crank_mass   = CRANK_DENSITY * math.pi * crank_radius**2 * (2.0 * crank_width)
crank_I_axial = 0.5 * crank_mass * crank_radius**2
crank_I_trans = (1.0 / 12.0) * crank_mass * (3.0 * crank_radius**2 + (2.0 * crank_width)**2)

crank = chrono.ChBody()
crank.SetMass(crank_mass)
crank.SetInertiaXX(chrono.ChVector3d(crank_I_trans, crank_I_trans, crank_I_axial))
crank.SetPos(CRANK_PIVOT)       # COM at the pivot (balanced disk)
crank.EnableCollision(False)
sys.AddBody(crank)

crank_disk = chrono.ChVisualShapeCylinder(crank_radius, 2.0 * crank_width)
crank_disk.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_disk)  # Z-axis cylinder; no offset needed for disk at pivot

crank_arm = chrono.ChVisualShapeCylinder(0.02, CRANK_LENGTH)
# Arm mid-point at (+CRANK_LENGTH/2, 0, 0) in body-local frame; rotate Z→X with QuatFromAngleY
crank.AddVisualShape(
    crank_arm,
    chrono.ChFramed(
        chrono.ChVector3d(CRANK_LENGTH / 2.0, 0.0, 0.0),
        chrono.QuatFromAngleY(chrono.CH_PI_2)
    )
)

# --- Connecting rod ---
# At t=0 the rod lies along world X (body-local X = world X, so SetRot = QUNIT)
rod_radius = 0.02
rod_mass   = ROD_DENSITY * math.pi * rod_radius**2 * ROD_LENGTH
rod_I_axial = 0.5 * rod_mass * rod_radius**2
rod_I_long  = (1.0 / 12.0) * rod_mass * (3.0 * rod_radius**2 + ROD_LENGTH**2)

rod = chrono.ChBody()
rod.SetMass(rod_mass)
rod.SetInertiaXX(chrono.ChVector3d(rod_I_long, rod_I_axial, rod_I_long))
rod.SetPos(ROD_MID_POS)
rod.SetRot(chrono.QUNIT)        # body-local X aligned with world X at t=0
rod.EnableCollision(False)
sys.AddBody(rod)

rod_cyl = chrono.ChVisualShapeCylinder(rod_radius, ROD_LENGTH)
rod_cyl.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
# Step 2 formula: rotate default Z-axis cylinder to body-local X (QuatFromAngleY(pi/2))
rod.AddVisualShape(
    rod_cyl,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))
)

# --- Piston ---
piston = chrono.ChBodyEasyBox(0.2, 0.2, 0.15, PISTON_DENSITY, True, False)
piston.SetPos(PISTON_INIT_POS)
piston.EnableCollision(False)
sys.Add(piston)
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.8))
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))

# === Joints / constraints ===
# Ground-truth slider-crank topology (Rule 6):
#   crank ↔ floor  : ChLinkMotorRotationSpeed (full motor-link — NO extra revolute)
#   crank ↔ rod    : ChLinkLockRevolute (crank pin)
#   rod   ↔ piston : ChLinkLockRevolute (wrist pin)
#   piston ↔ floor : ChLinkLockPrismatic (slides along X)
# Mechanism is in the XY plane → hinge axis = world Z → joint frame = QUNIT (Rule 8 special case)

# Motor: drives crank at constant speed about world Z through pivot
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

# Crank-pin revolute (body-local form): crank far end → rod near end
# crank local: (+CRANK_LENGTH, 0, 0);  rod local: (-ROD_LENGTH/2, 0, 0)
crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_LENGTH, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(crank_rod_joint)

# Wrist-pin revolute (body-local form): rod far end → piston centre
# rod local: (+ROD_LENGTH/2, 0, 0);  piston local: (0, 0, 0)
rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(rod_piston_joint)

# Prismatic guide: piston slides along world X on the fixed floor
# ChLinkLockPrismatic local +Z = sliding axis → Q_ROTATE_Z_TO_X for X-axis guide (Rule 9)
piston_floor_joint = chrono.ChLinkLockPrismatic()
piston_floor_joint.Initialize(
    piston, floor,
    chrono.ChFramed(PISTON_INIT_POS, chrono.Q_ROTATE_Z_TO_X)
)
sys.AddLink(piston_floor_joint)

# === Visualization ===
# Full Irrlicht scene: window config → Initialize → sky + camera + lights + grid (Y-up world)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up world convention; BEFORE Initialize
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism — PyChrono 9.0.x Irrlicht")
vis.Initialize()                                    # Initialize FIRST; scene elements come after
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.0, 1.5, 4.0),   # eye: above and in front of mechanism
    chrono.ChVector3d(1.5, 0.0, 0.0),   # look at: centre of stroke
)
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Main loop ===

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()   # cache: fetched once per step, reused in logging
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physical state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
