"""Slider-crank mechanism with spherical rod joints and a planar piston constraint.

Models a planar slider-crank linkage in PyChrono (rigid multi-body, ChSystemNSC,
no contact/collision — the topology is fully defined by joints and a driving motor).

Bodies:
  * ground          : fixed reference body (hosts the crank pivot and the planar guide).
  * crank           : disc/arm rotating about the world Z axis at the origin, driven
                      by a constant-speed rotational motor.
  * connecting rod  : links the crank pin to the piston.
  * piston          : the slider mass.

Topology (the defining feature of this model):
  * crank  <-> ground : revolute about world +Z  +  rotation-speed motor (the drive).
  * crank  <-> rod    : SPHERICAL (ball-and-socket) joint at the crank pin.
  * rod    <-> piston : SPHERICAL (ball-and-socket) joint at the piston wrist pin.
  * piston <-> ground : PLANAR (plane-plane) joint whose frame +Z is the world +Z,
                        so the piston is constrained to translate and rotate within
                        the world x-y plane (3 in-plane DOF: x, y, rotation about Z).

Gravity acts along world -Y, i.e. inside the x-y motion plane, so the planar joint
plane normal (world +Z) is perpendicular to gravity. Expected behavior: the crank
spins at a steady rate, the rod follows through the two ball joints, and the piston
oscillates back and forth predominantly along x within the x-y plane.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry, masses, drive, and time-stepping parameters
TIME_STEP = 1.0e-3          # physics step [s] (high-precision mechanism)
SIM_END = 6.0               # simulation duration [s]
RENDER_FPS = 50.0           # review-video frame rate [Hz]

CRANK_RADIUS = 1.0          # crank pin offset from rotation axis [m]
CRANK_THICK = 0.1           # crank disc thickness (visual) [m]
ROD_LENGTH = 3.0            # connecting-rod length between ball centers [m]
ROD_RADIUS = 0.05           # rod visual cylinder radius [m]
PISTON_SX = 0.4             # piston box size along x [m]
PISTON_SY = 0.4             # piston box size along y [m]
PISTON_SZ = 0.4             # piston box size along z [m]

CRANK_MASS = 2.0            # crank mass [kg]
ROD_MASS = 1.0              # connecting-rod mass [kg]
PISTON_MASS = 1.5           # piston mass [kg]

MOTOR_SPEED = 2.0 * math.pi  # crank angular speed [rad/s] (1 rev/s)
GRAVITY_Y = -9.81           # gravity along world -Y [m/s^2]

# === Derived positions === computed ONCE from the constants (precomputed once)
CRANK_PIVOT = chrono.ChVector3d(0.0, 0.0, 0.0)             # crank rotation axis (world origin)
CRANK_PIN0 = chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0)     # crank pin at start (crank angle 0)
# Piston wrist-pin x so that the rod length is satisfied at the start pose
# (crank pin at +x, piston on +x guide line through y = 0): x = r + rod_length.
PISTON_X0 = CRANK_RADIUS + ROD_LENGTH
PISTON_POS0 = chrono.ChVector3d(PISTON_X0, 0.0, 0.0)
ROD_MID0 = chrono.ChVector3d(0.5 * (CRANK_RADIUS + PISTON_X0), 0.0, 0.0)  # rod center at start

# Headless validation gate: fast, windowless physics check (see docs/codegen_rules.md)
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# === System & gravity === one ChSystemNSC; pure-joint mechanism, no collision system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, GRAVITY_Y, 0.0))

# === Bodies === ground + crank + connecting rod + piston (all built inline)
# Ground: fixed reference body that hosts the crank pivot and the planar guide.
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("ground")
sys.AddBody(ground)
# Thin visual guide marker on the ground along the slider axis for context.
guide_vis = chrono.ChVisualShapeBox(2.0 * PISTON_X0, 0.02, 0.02)
guide_vis.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground.AddVisualShape(guide_vis, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT))

# Crank: disc rotating about world +Z at the origin. Manual ChBody (it rotates).
crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(chrono.ChVector3d(0.5 * CRANK_MASS * CRANK_RADIUS ** 2,
                                     0.5 * CRANK_MASS * CRANK_RADIUS ** 2,
                                     0.5 * CRANK_MASS * CRANK_RADIUS ** 2))
crank.SetPos(CRANK_PIVOT)
crank.SetName("crank")
crank.EnableCollision(False)
sys.AddBody(crank)
# Crank disc visual (cylinder axis along body-local Z = world Z).
crank_disc = chrono.ChVisualShapeCylinder(CRANK_RADIUS * 0.6, CRANK_THICK)
crank_disc.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
crank.AddVisualShape(crank_disc, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
# Crank arm out to the pin (thin box from center to pin, along body-local +X).
crank_arm = chrono.ChVisualShapeBox(CRANK_RADIUS, 0.12, CRANK_THICK)
crank_arm.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
crank.AddVisualShape(crank_arm, chrono.ChFramed(chrono.ChVector3d(0.5 * CRANK_RADIUS, 0.0, 0.0), chrono.QUNIT))

# Connecting rod: links crank pin to piston wrist pin. Manual ChBody (it moves/rotates).
rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(chrono.ChVector3d(1.0e-3,
                                   (1.0 / 12.0) * ROD_MASS * ROD_LENGTH ** 2,
                                   (1.0 / 12.0) * ROD_MASS * ROD_LENGTH ** 2))
rod.SetPos(ROD_MID0)
rod.SetRot(chrono.QUNIT)  # rod axis along world +X at the start pose
rod.SetName("rod")
rod.EnableCollision(False)
sys.AddBody(rod)
# Rod visual: cylinder along body-local X (Step-2 visual offset rotates local Z -> X).
rod_cyl = chrono.ChVisualShapeCylinder(ROD_RADIUS, ROD_LENGTH)
rod_cyl.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Piston: the slider mass. Manual ChBody so it can rotate within the plane.
piston = chrono.ChBody()
piston.SetMass(PISTON_MASS)
piston.SetInertiaXX(chrono.ChVector3d((1.0 / 12.0) * PISTON_MASS * (PISTON_SY ** 2 + PISTON_SZ ** 2),
                                      (1.0 / 12.0) * PISTON_MASS * (PISTON_SX ** 2 + PISTON_SZ ** 2),
                                      (1.0 / 12.0) * PISTON_MASS * (PISTON_SX ** 2 + PISTON_SY ** 2)))
piston.SetPos(PISTON_POS0)
piston.SetName("piston")
piston.EnableCollision(False)
sys.AddBody(piston)
piston_box = chrono.ChVisualShapeBox(PISTON_SX, PISTON_SY, PISTON_SZ)
piston_box.SetColor(chrono.ChColor(0.3, 0.7, 0.3))
piston.AddVisualShape(piston_box, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# === Joints / constraints === revolute+motor drive, two spherical joints, one planar joint
# Crank <-> ground: revolute hinge about world +Z (local +Z = world +Z => QUNIT frame).
crank_to_ground = chrono.ChLinkLockRevolute()
crank_to_ground.Initialize(crank, ground, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
sys.AddLink(crank_to_ground)

# Drive motor: constant rotation speed about the same world +Z pivot axis.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(CRANK_PIVOT, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# Crank <-> rod: SPHERICAL (ball-and-socket) at the crank pin (world point CRANK_PIN0).
# 5-arg form: connection points are the crank pin (crank-local +X at the radius)
# and the rod near end (rod-local -ROD_LENGTH/2 along +X).
crank_rod_ball = chrono.ChLinkLockSpherical()
crank_rod_ball.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0), chrono.QUNIT),    # crank pin (local)
    chrono.ChFramed(chrono.ChVector3d(-0.5 * ROD_LENGTH, 0.0, 0.0), chrono.QUNIT),  # rod near end (local)
)
sys.AddLink(crank_rod_ball)

# Rod <-> piston: SPHERICAL (ball-and-socket) at the piston wrist pin.
rod_piston_ball = chrono.ChLinkLockSpherical()
rod_piston_ball.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(0.5 * ROD_LENGTH, 0.0, 0.0), chrono.QUNIT),  # rod far end (local)
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),               # piston center (local)
)
sys.AddLink(rod_piston_ball)

# Piston <-> ground: PLANAR (plane-plane) joint. The constraint plane normal is the
# joint frame +Z; with QUNIT the normal is world +Z, so the piston is free to
# translate and rotate within the world x-y plane (and is constrained out of it).
piston_to_ground = chrono.ChLinkLockPlanar()
piston_to_ground.Initialize(piston, ground, chrono.ChFramed(PISTON_POS0, chrono.QUNIT))
sys.AddLink(piston_to_ground)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)  # gravity along -Y -> Y is vertical
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Slider-Crank: spherical rod joints + planar piston")
    vis.Initialize()                                   # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, 2.0, 8.0), chrono.ChVector3d(2.0, 0.0, 0.0))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(2.0, -1.5, 0.0),
                                   chrono.QuatFromAngleX(chrono.CH_PI_2)),  # grid in x-z under the mechanism
                chrono.ChColor(0.4, 0.4, 0.4))

# === Derived run/render parameters === precomputed once, never recomputed in the loop
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating

# === Main loop === render-cadence outer loop; physics + CSV logging in the inner batch
os.makedirs("frames", exist_ok=True)   # guard against missing review-frame output dir
os.makedirs("cam", exist_ok=True)      # motion_log.csv lives under cam/

# cache: bound methods/handles fetched once and reused every step (avoid per-step getattr)
get_time = sys.GetChTime               # cache: system clock getter
step = sys.DoStepDynamics              # cache: integrator stepper
piston_pos = piston.GetPos            # cache: piston pose getter, reused each step
piston_vel = piston.GetPosDt          # cache: piston velocity getter, reused each step
crank_angle = motor.GetMotorAngle      # cache: integrated crank angle getter
crank_omega = motor.GetMotorAngleDt    # cache: crank angular speed getter

time_hist = []
piston_x_hist = []
piston_vx_hist = []
angle_hist = []

data_file = None
motion_file = None
try:
    data_file = open("simulation_data.csv", "w", newline="")        # main physics log
    motion_file = open("cam/motion_log.csv", "w", newline="")        # per-body motion contract
    data_writer = csv.writer(data_file)
    motion_writer = csv.writer(motion_file)
    data_writer.writerow(["time", "crank_angle", "crank_omega",
                          "piston_x", "piston_y", "piston_vx", "piston_vy"])
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    frame = 0
    while (HEADLESS or vis.Run()) and get_time() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1
        for _ in range(render_every):
            t = get_time()
            p = piston_pos()
            v = piston_vel()
            ang = crank_angle()
            om = crank_omega()
            data_writer.writerow([t, ang, om, p.x, p.y, v.x, v.y])
            cp = crank.GetPos(); cv = crank.GetPosDt()
            rp = rod.GetPos(); rv = rod.GetPosDt()
            motion_writer.writerow([t, "crank", cp.x, cp.y, cp.z, cv.x, cv.y, cv.z])
            motion_writer.writerow([t, "rod", rp.x, rp.y, rp.z, rv.x, rv.y, rv.z])
            motion_writer.writerow([t, "piston", p.x, p.y, p.z, v.x, v.y, v.z])
            time_hist.append(t)
            piston_x_hist.append(p.x)
            piston_vx_hist.append(v.x)
            angle_hist.append(ang)
            step(TIME_STEP)
            if get_time() >= run_end:
                break
except (OSError, IOError) as exc:           # disk full / permission error on CSV I/O
    import traceback
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush + close any open writers so partial output survives a mid-run error
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot piston motion vs crank angle to simulation_timeseries.png
if time_hist:
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ax1.plot(angle_hist, piston_x_hist, "b-")
    ax1.set(ylabel="piston x [m]", title="Slider-crank piston motion (spherical + planar joints)")
    ax1.grid(True)
    ax2.plot(angle_hist, piston_vx_hist, "r--")
    ax2.set(xlabel="crank angle [rad]", ylabel="piston vx [m/s]")
    ax2.grid(True)
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print(f"done: {len(time_hist)} steps logged, t_end={time_hist[-1] if time_hist else 0.0:.3f}s")
