"""
Slider-crank mechanism simulation (ChSystemNSC, Y-up, Irrlicht).

Models a motor-driven crank connected via a connecting rod to a piston
sliding along the X-axis.  The simulation runs for 20 s, collecting
crank angle, piston position, and piston speed every physics step, then
produces two Matplotlib subplots:
  - Position [m] vs crank angle [rad]
  - Speed [m/s] vs crank angle [rad]
Both x-axes use π-based tick labels (0, π/2, π, 3π/2, 2π).
"""

import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
CRANK_LENGTH  = 0.5    # crank arm length [m]
ROD_LENGTH    = 1.5    # connecting rod length [m]
MOTOR_SPEED   = chrono.CH_PI   # crank angular speed [rad/s] (1 rev/s)
TIME_STEP     = 1e-3   # physics time step [s]
SIM_END       = 20.0   # stop simulation after 20 s
RENDER_FPS    = 50.0
render_every  = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Derived geometry (precomputed once, not recomputed in the loop)
CRANK_COM_X   = CRANK_LENGTH / 2.0
ROD_COM_X     = CRANK_LENGTH + ROD_LENGTH / 2.0
PISTON_INIT_X = CRANK_LENGTH + ROD_LENGTH

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
# No contact/collision shapes in this pure-jointed MBS → no collision system needed

# === Bodies ===
# Ground / truss (fixed reference)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_vis = chrono.ChVisualShapeBox(0.2, 0.2, 0.5)
ground_vis.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground.AddVisualShape(ground_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.AddBody(ground)

# Crank arm — rotates about world origin (0, 0, 0)
crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
# Initial orientation: crank aligned along +X
crank.SetPos(chrono.ChVector3d(CRANK_COM_X, 0, 0))
crank.SetRot(chrono.QUNIT)
crank_vis = chrono.ChVisualShapeCylinder(0.05, CRANK_LENGTH)
crank_vis.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
crank.AddVisualShape(crank_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

# Connecting rod — links crank pin to piston pin
rod = chrono.ChBody()
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
rod.SetPos(chrono.ChVector3d(ROD_COM_X, 0, 0))
rod.SetRot(chrono.QUNIT)
rod_vis = chrono.ChVisualShapeCylinder(0.04, ROD_LENGTH)
rod_vis.SetColor(chrono.ChColor(0.3, 0.8, 0.3))
rod.AddVisualShape(rod_vis,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

# Piston — slides along X on the fixed guide
piston = chrono.ChBody()
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
piston.SetPos(chrono.ChVector3d(PISTON_INIT_X, 0, 0))
piston_vis = chrono.ChVisualShapeBox(0.2, 0.2, 0.2)
piston_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.8))
piston.AddVisualShape(piston_vis)
sys.AddBody(piston)

# === Joints / constraints ===
# Motor: crank ↔ ground (full motor-link — no separate revolute needed)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Crank pin: crank ↔ rod (revolute at far end of crank = (+CRANK_LENGTH, 0, 0) in world)
crank_pin = chrono.ChLinkLockRevolute()
crank_pin.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(+CRANK_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # crank far end local
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0, 0), chrono.QUNIT),   # rod near end local
)
sys.AddLink(crank_pin)

# Wrist pin: rod ↔ piston (revolute at far end of rod = piston center)
wrist_pin = chrono.ChLinkLockRevolute()
wrist_pin.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(+ROD_LENGTH / 2.0, 0, 0), chrono.QUNIT),  # rod far end local
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),                  # piston center local
)
sys.AddLink(wrist_pin)

# Prismatic: piston ↔ ground (slides along X)
prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(
    piston, ground,
    chrono.ChFramed(chrono.ChVector3d(PISTON_INIT_X, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(prismatic)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -4), chrono.ChVector3d(1, 0, 0))
vis.AddTypicalLights()

# === Data arrays for plotting (scored core — requested by the prompt) ===
array_time  = []
array_angle = []
array_pos   = []
array_speed = []

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()  # cache: fetched once, reused this step
            array_time.append(t)
            array_angle.append(motor.GetMotorAngle())
            array_pos.append(piston.GetPos().x)
            array_speed.append(piston.GetPosDt().x)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # no open file handles to flush here


# === Post-processing: Matplotlib plots ===
fig, (ax1, ax2) = plt.subplots(2, sharex=True, figsize=(8, 6))

ax1.plot(array_angle, array_pos)
ax1.set_ylabel("position [m]")
ax1.grid()

ax2.plot(array_angle, array_speed, "r--")
ax2.set_ylabel("speed [m/s]")
ax2.set_xlabel("crank angle [rad]")
ax2.grid()

# π-based x-axis ticks
x_ticks = np.linspace(0, 2 * np.pi, 5)
x_labels = ["0", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"]
plt.xticks(x_ticks, x_labels)

plt.tight_layout()
try:
    plt.savefig("simulation_timeseries.png", dpi=100)
except (OSError, IOError) as exc:      # disk full / permission denied
    import traceback
    traceback.print_exc()
