"""
Gear train simulation using PyChrono 9.0.x with Irrlicht visualization.

Models a two-gear meshing system (gearA driven by a motor, gearB driven by gear coupling)
mounted on a fixed truss backing plate. The motor rotates gearA at a constant angular speed,
and the ChLinkLockGear constraint transmits rotation to gearB with the correct ratio.

System: ChSystemNSC (pure jointed MBS — no contact between bodies, no collision system needed)
Bodies: truss (fixed backing plate), gearA (driving gear, radius=1.5), gearB (driven gear, radius=3.5)
Mechanism: ChLinkMotorRotationSpeed drives gearA at 3 rad/s; ChLinkLockGear couples gearA to gearB;
           ChLinkLockRevolute anchors gearB to the truss at its pivot.
Expected behavior: gearA rotates at 3 rad/s; gearB rotates opposite direction at
                   (radA/radB) * 3 = (1.5/3.5)*3 ≈ 1.286 rad/s.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
# Gear geometry (modified per specification)
RAD_A = 1.5        # radius of driving gear A (m)
RAD_B = 3.5        # radius of driven gear B (m)
INTERAXIS = RAD_A + RAD_B   # center-to-center = 5.0 m
GEAR_THICKNESS = 0.4         # thickness of gear disc visuals

# Truss dimensions (modified: 15 x 8 x 2, was 20 x 10 x 2)
TRUSS_X = 15.0
TRUSS_Y = 8.0
TRUSS_Z = 2.0

# Motor speed (modified: 3 rad/s, was 6)
MOTOR_SPEED = 3.0

# Shaft visual (modified: radius = RAD_A * 0.3 = 0.45, length = 10, was 13)
SHAFT_RADIUS = RAD_A * 0.3  # 0.45 m
SHAFT_LENGTH = 10.0

# GearB Z position (modified: -2, was -1); gearA stays at -1 (original)
GEAR_A_Z = -1.0
GEAR_B_Z = -2.0

# Truss positioned as a backing plate behind the gears (at Z = +0.5 from gear axis)
# Gears are at negative Z; truss backing is at positive Z side of the gear system
# Actually, gears at Z=-1 and Z=-2 are behind the truss at Z=0.
# The camera should look from behind (negative Z), pointing in +Z direction.
TRUSS_Z_POS = 0.0   # center of truss slab at Z=0; gears protrude at Z=-1, -2

# Simulation timing
TIME_STEP = 1e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
# Pure jointed MBS — no contact; omit SetCollisionSystemType
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up convention

# === Bodies ===

# Contact material (NSC) — required by EasyBox/EasyCylinder constructors
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)

# --- Truss (fixed support backing plate, modified: 15 x 8 x 2) ---
truss = chrono.ChBodyEasyBox(TRUSS_X, TRUSS_Y, TRUSS_Z, 1000.0, True, False, mat)
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(INTERAXIS / 2.0, 0.0, TRUSS_Z_POS))
truss.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
sys.AddBody(truss)

# --- Gear A (driving gear, radius=1.5) at (0, 0, -1) ---
# ChAxis_Z: cylinder axis along Z, so circular face is in XY plane (visible from +Z/-Z)
gear_a = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z,
    RAD_A,
    GEAR_THICKNESS,
    1500.0,
    True,
    False,
    mat
)
gear_a.SetPos(chrono.ChVector3d(0.0, 0.0, GEAR_A_Z))
gear_a.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.5, 0.1))
sys.AddBody(gear_a)

# --- Gear B (driven gear, radius=3.5) at (5, 0, -2) per modified specification ---
gear_b = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z,
    RAD_B,
    GEAR_THICKNESS,
    1500.0,
    True,
    False,
    mat
)
gear_b.SetPos(chrono.ChVector3d(INTERAXIS, 0.0, GEAR_B_Z))
gear_b.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.5, 0.9))
sys.AddBody(gear_b)

# --- Shaft visual (visual cylinder on gearA axis, modified: r=RAD_A*0.3, L=10) ---
# ChVisualShapeCylinder's default axis is Y; rotate to align with Z
shaft_vis = chrono.ChVisualShapeCylinder(SHAFT_RADIUS, SHAFT_LENGTH)
shaft_vis.SetColor(chrono.ChColor(0.25, 0.25, 0.25))
q_yto_z = chrono.QuatFromAngleAxis(math.pi / 2.0, chrono.VECT_X)  # rotate Y axis → Z axis
gear_a.AddVisualShape(shaft_vis, chrono.ChFramed(chrono.VNULL, q_yto_z))

# === Joints / constraints ===

# Hinge axis is world Z; ChLinkLockRevolute local +Z = world +Z with QUNIT

# --- Motor: drives gearA relative to truss at constant speed (full motor-link, no extra revolute) ---
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    gear_a, truss,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, GEAR_A_Z), chrono.QUNIT)
)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))  # 3 rad/s
sys.AddLink(link_motor)

# --- Revolute: anchors gearB to truss at its pivot ---
link_rev_b = chrono.ChLinkLockRevolute()
link_rev_b.Initialize(
    gear_b, truss,
    chrono.ChFramed(chrono.ChVector3d(INTERAXIS, 0.0, GEAR_B_Z), chrono.QUNIT)
)
sys.AddLink(link_rev_b)

# --- Gear coupling: enforces meshing ratio and phase between gearA and gearB ---
gear_link = chrono.ChLinkLockGear()
gear_link.Initialize(gear_a, gear_b, chrono.ChFramed())
gear_link.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QUNIT))   # shaft1 along world Z
gear_link.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QUNIT))   # shaft2 along world Z
gear_link.SetTransmissionRatio(RAD_A / RAD_B)  # 1.5/3.5 ≈ 0.4286
gear_link.SetEnforcePhase(True)
sys.AddLink(gear_link)

# === Visualization ===
# Camera positioned at negative Z, looking toward +Z so gears (at Z=-1,-2) are visible
# in front of the truss backing plate (at Z=0)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train — radA=1.5, radB=3.5, motor=3 rad/s")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()  # Initialize FIRST (Irrlicht: scene elements added AFTER)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(INTERAXIS / 2.0, 3.0, -15.0),  # behind gears (negative Z), looking +Z
    chrono.ChVector3d(INTERAXIS / 2.0, 0.0, -1.5)    # target: gear center region
)
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 30, 30,
    chrono.ChCoordsysd(chrono.ChVector3d(INTERAXIS / 2.0, -4.5, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Review-only recording setup ===


# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
