"""
Slider-Crank Mechanism with Data Collection and Matplotlib Plotting.

System type: ChSystemNSC (pure jointed MBS, no contact/collision).
Bodies: fixed floor/truss, crank (rotating), connecting rod, piston (sliding).
Motor: ChLinkMotorRotationSpeed drives the crank at constant angular speed.
Constraints: crank-floor (motor), crank-rod (revolute), rod-piston (revolute),
             piston-floor (prismatic along X).

Simulation collects arrays of time, crank angle, piston position, and piston speed
over 20 seconds, then plots position vs crank angle and speed vs crank angle using
Matplotlib with numpy-based pi-tick formatting.
"""

# === Imports ===
import math
import numpy as np
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants: geometry and physics ===
CRANK_LENGTH = 0.2          # crank arm radius [m]
ROD_LENGTH   = 0.5          # connecting rod length [m]
PISTON_MASS  = 1.0          # piston mass [kg]
CRANK_MASS   = 0.5          # crank mass [kg]
ROD_MASS     = 0.3          # connecting rod mass [kg]
CRANK_SPEED  = math.pi      # crank angular speed [rad/s] (≈ 0.5 rev/s)

TIME_STEP  = 1e-3           # integration step [s]
SIM_END    = 20.0           # simulation end time [s]; prompt specifies 20 s
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Derived geometry: crank pivot at origin, piston slides along X
CRANK_PIVOT  = chrono.ChVector3d(0, 0, 0)
PISTON_GUIDE_Y = 0.0        # piston center Y = 0 (slides along X)

# === System and gravity (Y-up, pure jointed MBS — no collision system needed) ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Bodies ===

# Ground / truss (fixed)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
# Visual: small box at origin to mark the pivot
pivot_vis = chrono.ChVisualShapeBox(0.05, 0.05, 0.1)
pivot_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(pivot_vis, chrono.ChFramed(CRANK_PIVOT))
# Visual: guide rail for piston
rail_vis = chrono.ChVisualShapeBox(ROD_LENGTH + CRANK_LENGTH + 0.1, 0.02, 0.05)
rail_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(rail_vis, chrono.ChFramed(chrono.ChVector3d(
    (ROD_LENGTH + CRANK_LENGTH) / 2, 0, 0)))
sys.Add(ground)

# Crank body (rotates about pivot; modeled as a link in the XY plane)
crank = chrono.ChBody()
crank.SetName("crank")
crank.SetMass(CRANK_MASS)
Ic = (1.0 / 3.0) * CRANK_MASS * CRANK_LENGTH**2   # thin rod rotated about one end
crank.SetInertiaXX(chrono.ChVector3d(Ic, Ic, Ic))
# Initial position: crank pointing right (+X)
crank_mid = chrono.ChVector3d(CRANK_LENGTH / 2, 0, 0)
crank.SetPos(crank_mid)
crank.SetRot(chrono.QUNIT)
# Visual cylinder along body-local X using two-step convention
crank_cyl = chrono.ChVisualShapeCylinder(0.02, CRANK_LENGTH)
crank_cyl.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_cyl, chrono.ChFramed(chrono.VNULL,
                                                chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.Add(crank)

# Connecting rod body (links crank pin to piston wrist pin)
# Initial: crank at 0°, so crank pin at (CRANK_LENGTH, 0); piston at (CRANK_LENGTH + ROD_LENGTH, 0)
crank_pin_pos  = chrono.ChVector3d(CRANK_LENGTH, 0, 0)
piston_wrist_pos = chrono.ChVector3d(CRANK_LENGTH + ROD_LENGTH, 0, 0)
rod_mid_pos = chrono.ChVector3d(
    (CRANK_LENGTH + CRANK_LENGTH + ROD_LENGTH) / 2, 0, 0)

rod = chrono.ChBody()
rod.SetName("rod")
rod.SetMass(ROD_MASS)
Ir = (1.0 / 12.0) * ROD_MASS * ROD_LENGTH**2   # thin rod rotated about center
rod.SetInertiaXX(chrono.ChVector3d(Ir, Ir, Ir))
rod.SetPos(rod_mid_pos)
rod.SetRot(chrono.QUNIT)
rod_cyl = chrono.ChVisualShapeCylinder(0.015, ROD_LENGTH)
rod_cyl.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL,
                                            chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.Add(rod)

# Piston body (slides along X axis)
piston = chrono.ChBody()
piston.SetName("piston")
piston.SetMass(PISTON_MASS)
Ip = (1.0 / 12.0) * PISTON_MASS * (0.06**2 + 0.06**2)
piston.SetInertiaXX(chrono.ChVector3d(Ip, Ip, Ip))
piston.SetPos(piston_wrist_pos)
piston.SetRot(chrono.QUNIT)
piston_box = chrono.ChVisualShapeBox(0.08, 0.06, 0.06)
piston_box.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
piston.AddVisualShape(piston_box)
sys.Add(piston)

# === Joints / constraints ===

# 1. Crank ↔ ground: motor-driven rotation (ChLinkMotorRotationSpeed is a full motor-link)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

# 2. Crank ↔ rod: revolute at crank-pin
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_LENGTH / 2, 0, 0), chrono.QUNIT),  # local on crank (far end)
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2, 0, 0), chrono.QUNIT),   # local on rod (near end)
)
sys.AddLink(joint_crank_rod)

# 3. Rod ↔ piston: revolute at wrist-pin
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2, 0, 0), chrono.QUNIT),    # local on rod (far end)
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),                  # local on piston (center)
)
sys.AddLink(joint_rod_piston)

# 4. Piston ↔ ground: prismatic along X (local +Z must map to world +X → use Q_ROTATE_Z_TO_X)
joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(
    piston, ground,
    chrono.ChFramed(piston_wrist_pos, chrono.Q_ROTATE_Z_TO_X)
)
sys.AddLink(joint_piston_ground)

# === Data arrays (collected during simulation — prompt requirement) ===
array_time  = []   # simulation time [s]
array_angle = []   # crank angle [rad]
array_pos   = []   # piston X position [m]
array_speed = []   # piston X velocity [m/s]

# === Visualization — full Irrlicht block ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank: Piston Position and Speed vs Crank Angle")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()                                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.4, 1.2), chrono.ChVector3d(0.5, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(0.1, 0.1, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -0.05, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

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
            # Collect data arrays — prompt requirement
            crank_angle = motor.GetMotorAngle()
            piston_pos   = piston.GetPos().x
            piston_vel   = piston.GetPosDt().x
            array_time.append(t)
            array_angle.append(crank_angle)
            array_pos.append(piston_pos)
            array_speed.append(piston_vel)
            sys.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing — Matplotlib plots (prompt requirement) ===
# Plot piston position and speed vs crank angle with pi-based x-axis ticks.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

arr_angle = np.array(array_angle)
arr_pos   = np.array(array_pos)
arr_speed = np.array(array_speed)

# Pi-based tick positions and labels for x-axis
pi_ticks  = np.array([0, 0.5, 1.0, 1.5, 2.0]) * math.pi  # 0, π/2, π, 3π/2, 2π
pi_labels = ["0", "π/2", "π", "3π/2", "2π"]

# Normalise angle to [0, 2π] for plotting
arr_angle_mod = arr_angle % (2 * math.pi)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
fig.suptitle("Slider-Crank: Piston vs Crank Angle")

ax1.plot(arr_angle_mod, arr_pos, linewidth=0.8, color="steelblue")
ax1.set_xlabel("Crank angle [rad]")
ax1.set_ylabel("Piston position [m]")
ax1.set_xticks(pi_ticks)
ax1.set_xticklabels(pi_labels)
ax1.grid(True)

ax2.plot(arr_angle_mod, arr_speed, linewidth=0.8, color="darkorange")
ax2.set_xlabel("Crank angle [rad]")
ax2.set_ylabel("Piston speed [m/s]")
ax2.set_xticks(pi_ticks)
ax2.set_xticklabels(pi_labels)
ax2.grid(True)

plt.tight_layout()
try:
    plt.savefig("simulation_timeseries.png", dpi=100)
except (OSError, IOError) as exc:    # disk / permission error saving plot
    import traceback
    traceback.print_exc()
plt.close(fig)
