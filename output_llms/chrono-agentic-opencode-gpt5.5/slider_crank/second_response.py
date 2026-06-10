"""Motor-driven slider-crank mechanism in an NSC multi-body system.

The model contains a fixed floor/truss, a rotating crank, a connecting rod, and
a piston constrained to slide along a horizontal guide. A prescribed-speed motor
drives the crank while arrays collect crank angle, piston position, and piston
speed for the requested position-angle and speed-angle plots over 20 seconds.
"""

import math
import os
import traceback

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === direct demo-scale values for the planar mechanism
TIME_STEP = 1.0e-3
SIM_END = 20.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
CRANK_RADIUS = 1.0
ROD_LENGTH = 4.0
BODY_RADIUS = 0.08
BODY_DEPTH = 0.12
CRANK_SPEED = chrono.CH_PI
PISTON_X0 = CRANK_RADIUS + ROD_LENGTH

array_time = []
array_angle = []
array_pos = []
array_speed = []


# === System & Gravity === pure constrained MBS, no contact collision required
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === floor, crank, rod, piston, and visible guide are separate bodies
floor = chrono.ChBody()
floor.SetFixed(True)
sys.AddBody(floor)

crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
crank.SetPos(chrono.ChVector3d(CRANK_RADIUS / 2.0, 0.0, 0.0))
crank.SetRot(chrono.QUNIT)
crank.EnableCollision(False)
crank_shape = chrono.ChVisualShapeCylinder(BODY_RADIUS, CRANK_RADIUS)
crank_shape.SetColor(chrono.ChColor(0.9, 0.2, 0.2))
crank.AddVisualShape(crank_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(crank)

rod = chrono.ChBody()
rod.SetMass(1.0)
rod.SetInertiaXX(chrono.ChVector3d(0.2, 1.0, 1.0))
rod.SetPos(chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH / 2.0, 0.0, 0.0))
rod.SetRot(chrono.QUNIT)
rod.EnableCollision(False)
rod_shape = chrono.ChVisualShapeCylinder(BODY_RADIUS * 0.75, ROD_LENGTH)
rod_shape.SetColor(chrono.ChColor(0.2, 0.3, 0.9))
rod.AddVisualShape(rod_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
sys.AddBody(rod)

piston = chrono.ChBodyEasyBox(0.45, 0.30, 0.30, 1000.0, True, False)
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
piston.SetPos(chrono.ChVector3d(PISTON_X0, 0.0, 0.0))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
sys.AddBody(piston)

guide = chrono.ChBodyEasyBox(6.0, 0.04, 0.04, 1000.0, True, False)
guide.SetFixed(True)
guide.SetPos(chrono.ChVector3d(3.5, -0.28, 0.0))
guide.GetVisualShape(0).SetColor(chrono.ChColor(0.35, 0.35, 0.35))
sys.AddBody(guide)


# === Joints / Constraints === motor-crank, rod pins, and fixed piston guide
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    crank,
    rod,
    True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(joint_crank_rod)

joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    rod,
    piston,
    True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(joint_rod_piston)

joint_piston_floor = chrono.ChLinkLockPrismatic()
joint_piston_floor.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(PISTON_X0, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(joint_piston_floor)


# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider Crank")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.6, 3.0, 7.0), chrono.ChVector3d(2.6, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    16,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(2.5, -0.55, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.45, 0.45, 0.45),
)


# === Main Loop === render, collect requested arrays, and advance the mechanism
motor_cache = motor  # cache: motor angle queried every physics step
piston_cache = piston  # cache: piston state queried every physics step
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            angle = motor_cache.GetMotorAngle()
            piston_pos = piston_cache.GetPos().x
            piston_speed = piston_cache.GetPosDt().x
            array_time.append(sim_time)
            array_angle.append(math.fmod(angle, 2.0 * math.pi))
            array_pos.append(piston_pos)
            array_speed.append(piston_speed)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (OSError, IOError) as exc:  # disk or permission failure while writing outputs
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono solver/runtime failure or invalid state
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === requested matplotlib plots of piston state versus crank angle
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    ax1.plot(array_angle, array_pos)
    ax1.set(ylabel="position [m]")
    ax1.grid(True)
    ax2.plot(array_angle, array_speed, "r--")
    ax2.set(ylabel="speed [m/s]", xlabel="crank angle [rad]")
    ax2.grid(True)
    plt.xticks(
        np.linspace(0.0, 2.0 * np.pi, 5),
        ["0", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"],
    )
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=150)
    plt.close(fig)
except (OSError, RuntimeError, ValueError) as exc:  # plotting backend or output-file failure
    traceback.print_exc()
    raise
