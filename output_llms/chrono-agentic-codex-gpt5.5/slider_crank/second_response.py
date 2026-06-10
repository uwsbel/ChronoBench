"""Motor-driven slider-crank mechanism in a Y-up PyChrono NSC system.

The model contains a fixed floor/truss, a rotating crank, a connecting rod, and
a piston constrained to slide horizontally. The crank motor drives one full
rotation per second while the script records crank angle, piston position, and
piston speed for the requested position/speed plots over a 20 second run.
"""

import math
import traceback

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants === compact mechanism dimensions and run cadence
TIME_STEP = 1.0e-3
SIM_END = 20.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
CRANK_RADIUS = 0.4
ROD_LENGTH = 1.5
CRANK_SPEED = 2.0 * math.pi
PI_TICKS = np.linspace(0.0, 2.0 * math.pi, 5)
PI_LABELS = ["0", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"]


def make_link(name, length, radius, mass, center_x, center_y, angle):
    """Create a simple cylindrical link whose body-local X axis follows angle."""
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVector3d(0.1, 1.0, 1.0))
    body.SetPos(chrono.ChVector3d(center_x, center_y, 0.0))
    body.SetRot(chrono.QuatFromAngleZ(angle))
    body.EnableCollision(False)
    shape = chrono.ChVisualShapeCylinder(radius, length)
    body.AddVisualShape(shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
    return body


def write_requested_plots(array_angle, array_pos, array_speed):
    """Write the requested position/speed plots versus crank angle."""
    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    ax1.plot(array_angle, array_pos)
    ax1.set(ylabel="position [m]")
    ax1.grid(True)
    ax2.plot(array_angle, array_speed, "r--")
    ax2.set(ylabel="speed [m/s]", xlabel="crank angle [rad]")
    ax2.grid(True)
    plt.xticks(PI_TICKS, PI_LABELS)
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=150)
    plt.close(fig)


# === System & gravity === NSC mechanism without contact geometry
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))


# === Bodies === fixed support plus crank, rod, and sliding piston
floor = chrono.ChBody()
floor.SetName("floor")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
guide_shape = chrono.ChVisualShapeBox(CRANK_RADIUS + ROD_LENGTH + 0.6, 0.04, 0.08)
floor.AddVisualShape(
    guide_shape,
    chrono.ChFramed(chrono.ChVector3d((CRANK_RADIUS + ROD_LENGTH) / 2.0, -0.18, 0.0), chrono.QUNIT),
)
pivot_shape = chrono.ChVisualShapeBox(0.08, 0.42, 0.08)
floor.AddVisualShape(
    pivot_shape,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddBody(floor)

crank = make_link("crank", CRANK_RADIUS, 0.04, 1.0, CRANK_RADIUS / 2.0, 0.0, 0.0)
rod = make_link(
    "connecting_rod",
    ROD_LENGTH,
    0.03,
    1.0,
    CRANK_RADIUS + ROD_LENGTH / 2.0,
    0.0,
    0.0,
)
piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.12, 0.2, 1000.0)
piston.SetName("piston")
piston.SetMass(1.0)
piston.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))
piston.SetPos(chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH, 0.0, 0.0))
piston.EnableCollision(False)

sys.AddBody(crank)
sys.AddBody(rod)
sys.AddBody(piston)

crank_body = crank  # cache: reused by joints and data collection
rod_body = rod  # cache: reused by joints
piston_body = piston  # cache: reused in loop for position and velocity


# === Joints / constraints === canonical slider-crank topology
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank_body, floor, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sys.AddLink(motor)

crank_rod = chrono.ChLinkLockRevolute()
crank_rod.Initialize(
    crank_body,
    rod_body,
    True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(-ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(crank_rod)

rod_piston = chrono.ChLinkLockRevolute()
rod_piston.Initialize(
    rod_body,
    piston_body,
    True,
    chrono.ChFramed(chrono.ChVector3d(ROD_LENGTH / 2.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
)
sys.AddLink(rod_piston)

piston_guide = chrono.ChLinkLockPrismatic()
piston_guide.Initialize(
    piston_body,
    floor,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH, 0.0, 0.0), chrono.Q_ROTATE_Z_TO_X),
)
sys.AddLink(piston_guide)


# === Visualization === Irrlicht window with Y-up view and mechanism camera
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.9, 2.0, 3.2), chrono.ChVector3d(0.9, 0.0, 0.0))
vis.AddTypicalLights()


# === Main loop === collect requested arrays and record review frames when enabled
array_time = []
array_angle = []
array_pos = []
array_speed = []

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()  # cache: time reused for stop/log arrays
            piston_pos = piston_body.GetPos()  # cache: reused for x-position
            piston_vel = piston_body.GetPosDt()  # cache: reused for x-speed
            array_time.append(sim_time)
            array_angle.append(math.fmod(motor.GetMotorAngle(), 2.0 * math.pi))
            array_pos.append(piston_pos.x)
            array_speed.append(piston_vel.x)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output device or plot path guard
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === requested matplotlib plots
write_requested_plots(array_angle, array_pos, array_speed)
