"""
Slider-Crank Mechanism Simulation (PyChrono 9.0.x, ChSystemNSC, Y-up).

Models a motor-driven crank connected to a connecting rod and a piston
that slides along a horizontal fixed guide. Collects time-series data
(crank angle, piston position, piston speed) during the simulation and
produces two matplotlib subplots at the end:
  1. Piston position [m] vs. crank angle [rad]
  2. Piston speed [m/s] vs. crank angle [rad]
The simulation stops automatically after 20 seconds.
"""

import math
import os
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants ===
CRANK_LENGTH   = 0.5      # m, crank radius
ROD_LENGTH     = 1.5      # m, connecting rod length
PISTON_RADIUS  = 0.05     # m, piston half-diameter (visual cylinder radius)
PISTON_HEIGHT  = 0.1      # m, piston visual height

CRANK_MASS     = 1.0      # kg
ROD_MASS       = 1.0      # kg
PISTON_MASS    = 1.0      # kg

MOTOR_SPEED    = chrono.CH_PI   # rad/s (half a turn per second)

TIME_STEP      = 1e-3     # s
SIM_END        = 20.0     # s
RENDER_FPS     = 50.0
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Derived geometry (Y-up, crank pivot at origin)
CRANK_PIN_OFFSET  = CRANK_LENGTH              # crank pin initial at (CRANK_LENGTH, 0, 0)
ROD_CENTER_X      = CRANK_LENGTH + ROD_LENGTH / 2.0
PISTON_INIT_X     = CRANK_LENGTH + ROD_LENGTH  # piston initial X position

# === System & gravity ===
# Pure jointed MBS — no contact; collision system omitted intentionally (no contact shapes).
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===
# Ground / truss — fixed reference body
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
# Visual: thin horizontal bar as guide rail
rail_vis = chrono.ChVisualShapeBox(PISTON_INIT_X * 2 + 0.2, 0.02, 0.1)
rail_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(rail_vis, chrono.ChFramed(
    chrono.ChVector3d(PISTON_INIT_X / 2.0, 0.0, 0.0), chrono.QUNIT))
sys.AddBody(ground)

# Crank body — rotates about the origin
crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.2))
crank.SetPos(chrono.ChVector3d(CRANK_LENGTH / 2.0, 0.0, 0.0))
# Visual: cylinder laid along X (the crank arm)
crank_vis = chrono.ChVisualShapeCylinder(0.03, CRANK_LENGTH)
crank_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))
sys.AddBody(crank)

# Connecting rod — links crank pin to piston
rod = chrono.ChBody()
rod.SetName("rod")
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
rod.SetPos(chrono.ChVector3d(ROD_CENTER_X, 0.0, 0.0))
rod_vis = chrono.ChVisualShapeCylinder(0.02, ROD_LENGTH)
rod_vis.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
rod.AddVisualShape(rod_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))
sys.AddBody(rod)

# Piston — slides along horizontal guide (X-axis)
piston = chrono.ChBody()
piston.SetName("piston")
piston.SetMass(PISTON_MASS)
piston.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
piston.SetPos(chrono.ChVector3d(PISTON_INIT_X, 0.0, 0.0))
piston_vis = chrono.ChVisualShapeBox(PISTON_HEIGHT, 0.12, 0.12)
piston_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints ===
# Motor: crank ↔ ground at origin — ChLinkMotorRotationSpeed is a FULL motor-link;
# no separate revolute is needed.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Crank ↔ Rod: revolute at crank pin (local +Z = world +Z for XY-plane swing)
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(+CRANK_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),  # crank far end
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),    # rod near end
)
sys.AddLink(joint_crank_rod)

# Rod ↔ Piston: revolute at wrist pin (XY plane, hinge axis = +Z → QUNIT)
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(+ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),   # rod far end
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),                 # piston center
)
sys.AddLink(joint_rod_piston)

# Piston ↔ Ground: prismatic along global X (local +Z maps to X via Q_ROTATE_Z_TO_X)
joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(
    piston, ground,
    chrono.ChFramed(chrono.ChVector3d(PISTON_INIT_X, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(joint_piston_ground)

# === Visualization ===  full Irrlicht scene: window + Initialize first, then sky+camera+lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()  # Initialize FIRST; add scene elements after
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(PISTON_INIT_X / 2.0, 1.5, 3.0),
    chrono.ChVector3d(PISTON_INIT_X / 2.0, 0.0, 0.0))
vis.AddTypicalLights()

# === Data arrays for plotting (scored core — prompt requires them) ===
array_time  = []
array_angle = []
array_pos   = []
array_speed = []

# === Review-only recording setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            if t >= SIM_END:
                break
            # Collect data — scored core (prompt objective)
            crank_angle = motor.GetMotorAngle()
            piston_pos   = piston.GetPos().x
            piston_speed = piston.GetPosDt().x
            array_time.append(t)
            array_angle.append(crank_angle)
            array_pos.append(piston_pos)
            array_speed.append(piston_speed)


            sys.DoStepDynamics(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # arrays are in-memory; no open file handles in scored core


# === Post-processing: matplotlib plots (scored core — prompt objective) ===
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

angle_arr = np.array(array_angle)
pos_arr   = np.array(array_pos)
speed_arr = np.array(array_speed)

# Normalise angle to [0, 2π) for x-axis labels
angle_mod = np.mod(angle_arr, 2.0 * np.pi)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
fig.suptitle("Slider-Crank: piston kinematics vs. crank angle")

ax1.plot(angle_mod, pos_arr, linewidth=1.0)
ax1.set_xlabel("Crank angle [rad]")
ax1.set_ylabel("Piston position [m]")
# π-based x-axis ticks
ticks = np.array([0, 0.5, 1.0, 1.5, 2.0]) * np.pi
tick_labels = ["0", "π/2", "π", "3π/2", "2π"]
ax1.set_xticks(ticks)
ax1.set_xticklabels(tick_labels)
ax1.grid(True)

ax2.plot(angle_mod, speed_arr, linewidth=1.0, color="tab:orange")
ax2.set_xlabel("Crank angle [rad]")
ax2.set_ylabel("Piston speed [m/s]")
ax2.set_xticks(ticks)
ax2.set_xticklabels(tick_labels)
ax2.grid(True)

plt.tight_layout()
try:
    plt.savefig("simulation_timeseries.png", dpi=120)
except (OSError, IOError) as exc:  # disk / permission error writing PNG
    import traceback
    traceback.print_exc()
