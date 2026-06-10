"""
Slider-crank mechanism simulation (PyChrono 9.0.x, Irrlicht renderer).

Model
-----
A planar crank-rod-slider linkage modeled with a Non-Smooth-Contact system
(ChSystemNSC). Gravity acts along -Y; the mechanism lies in the world XY plane,
so every revolute hinge axis is world +Z (perpendicular to the motion plane).

Bodies
------
- ground      : fixed reference body carrying the crank pivot and the slider guide.
- crank        : disc/arm rotating about a fixed ground pivot at the origin, driven
                 by a constant-speed rotational motor.
- connecting_rod : links the crank pin to the slider via two revolute pins.
- slider (piston): translates along the world +X guide (prismatic to ground).

Topology (fixed-guide slider linkage checklist)
-----------------------------------------------
- crank-ground   : revolute (hinge axis +Z) + rotational-speed motor (actuation).
- crank-rod      : revolute (hinge axis +Z) at the crank pin.
- rod-slider     : revolute (hinge axis +Z) at the wrist pin.
- slider-ground  : prismatic, sliding axis world +X.

Expected behavior
-----------------
The motor spins the crank at a constant angular speed, so the crank angle grows
linearly with time. The slider oscillates back and forth along X with a roughly
sinusoidal position and velocity. The run integrates for 20 s, logs the crank
angle, slider position and slider speed every step, and at the end produces two
matplotlib subplots: slider position vs crank angle and slider speed vs crank
angle, with the angle axis ticked in multiples of pi.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for PNG output
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants (geometry, physics, run control) ===
TIME_STEP = 1.0e-3        # integration step [s] (high-precision mechanism)
SIM_END = 20.0            # stop the simulation after 20 seconds [s]
RENDER_FPS = 25.0         # review-frame cadence [frames/s]

CRANK_RADIUS = 1.0        # crank arm length, pivot -> crank pin [m]
ROD_LENGTH = 4.0          # connecting-rod length, crank pin -> wrist pin [m]
CRANK_THICK = 0.2         # crank/rod visual cross-section [m]
SLIDER_SIZE = 0.6         # slider cube edge [m]

CRANK_MASS = 2.0          # crank mass [kg]
ROD_MASS = 1.0            # connecting-rod mass [kg]
SLIDER_MASS = 1.0         # slider mass [kg]

MOTOR_SPEED = 1.0         # crank angular speed [rad/s] (constant-speed motor)

# Derived constants (precomputed once, never recomputed in the loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
ROD_INERTIA = (1.0 / 12.0) * ROD_MASS * (ROD_LENGTH ** 2)     # rod inertia about transverse axis
CRANK_INERTIA = 0.5 * CRANK_MASS * (CRANK_RADIUS ** 2)         # crank inertia about spin axis

# World anchor points (motion plane is XY, gravity along -Y)
PIVOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)                   # crank-ground revolute
CRANK_PIN_POS = chrono.ChVector3d(CRANK_RADIUS, 0.0, 0.0)      # crank-rod revolute (crank at angle 0)
WRIST_POS = chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH, 0.0, 0.0)  # rod-slider revolute

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run

# === System & gravity === one NSC system, gravity along -Y (planar XY mechanism)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))
# Pure joint-driven linkage with no contact -> do NOT enable a collision system.

# === Bodies === ground + crank + connecting rod + slider, all built inline
# Ground: fixed reference for the crank pivot and the slider guide.
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)
ground_guide = chrono.ChVisualShapeCylinder(0.05, CRANK_RADIUS + ROD_LENGTH + 1.0)
ground.AddVisualShape(ground_guide, chrono.ChFramed(
    chrono.ChVector3d((CRANK_RADIUS + ROD_LENGTH) * 0.5, 0.0, -0.4),
    chrono.QuatFromAngleY(chrono.CH_PI_2)))  # thin rail marking the slider guide axis (+X)

# Crank: rotates about the ground pivot; body-local X aligned with world X at angle 0.
crank = chrono.ChBody()
crank.SetMass(CRANK_MASS)
crank.SetInertiaXX(chrono.ChVector3d(CRANK_INERTIA, CRANK_INERTIA, CRANK_INERTIA))
crank.SetPos(chrono.ChVector3d(CRANK_RADIUS * 0.5, 0.0, 0.0))  # midpoint pivot->pin
crank.SetRot(chrono.QUNIT)  # body-local X already along world X
crank.EnableCollision(False)
sys.AddBody(crank)
crank_vis = chrono.ChVisualShapeCylinder(CRANK_THICK * 0.5, CRANK_RADIUS)
crank_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
crank.AddVisualShape(crank_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Connecting rod: spans crank pin -> wrist pin; body-local X along world X at start.
rod = chrono.ChBody()
rod.SetMass(ROD_MASS)
rod.SetInertiaXX(chrono.ChVector3d(ROD_INERTIA, ROD_INERTIA, ROD_INERTIA))
rod.SetPos(chrono.ChVector3d(CRANK_RADIUS + ROD_LENGTH * 0.5, 0.0, 0.0))  # midpoint pin->wrist
rod.SetRot(chrono.QUNIT)
rod.EnableCollision(False)
sys.AddBody(rod)
rod_vis = chrono.ChVisualShapeCylinder(CRANK_THICK * 0.4, ROD_LENGTH)
rod_vis.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
rod.AddVisualShape(rod_vis, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Slider (piston): translates along the +X guide; starts at the wrist position.
slider = chrono.ChBody()
slider.SetMass(SLIDER_MASS)
slider.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
slider.SetPos(WRIST_POS)
slider.SetRot(chrono.QUNIT)
slider.EnableCollision(False)
sys.AddBody(slider)
slider_vis = chrono.ChVisualShapeBox(SLIDER_SIZE, SLIDER_SIZE, SLIDER_SIZE)
slider_vis.SetColor(chrono.ChColor(0.2, 0.7, 0.3))
slider.AddVisualShape(slider_vis)

# === Joints / constraints === fixed-guide slider linkage (all hinge axes world +Z)
# crank-ground revolute at the pivot (local +Z = world +Z, so QUNIT is correct).
joint_crank_ground = chrono.ChLinkLockRevolute()
joint_crank_ground.Initialize(crank, ground, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
sys.AddLink(joint_crank_ground)

# crank-ground motor: drives the crank at a constant angular speed about +Z.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(PIVOT_POS, chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(MOTOR_SPEED))
sys.AddLink(motor)

# crank-rod revolute at the crank pin.
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(crank, rod, chrono.ChFramed(CRANK_PIN_POS, chrono.QUNIT))
sys.AddLink(joint_crank_rod)

# rod-slider revolute at the wrist pin (fixed-guide linkage uses a pin here, NOT a prismatic).
joint_rod_slider = chrono.ChLinkLockRevolute()
joint_rod_slider.Initialize(rod, slider, chrono.ChFramed(WRIST_POS, chrono.QUNIT))
sys.AddLink(joint_rod_slider)

# slider-ground prismatic: sliding axis world +X. Frame local +Z must map onto +X.
joint_slider_ground = chrono.ChLinkLockPrismatic()
joint_slider_ground.Initialize(slider, ground, chrono.ChFramed(WRIST_POS, chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(joint_slider_ground)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity along -Y -> Y-up view
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Slider-Crank Mechanism")
    vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.5, 4.0, -9.0), chrono.ChVector3d(2.5, 0.0, 0.0))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(2.5, -1.0, 0.0),
                                   chrono.QuatFromAngleX(chrono.CH_PI_2)),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid (in XZ plane below)

# === Main loop === drive the crank, log angle/pos/speed each step, save review frames
os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)       # motion log lives under cam/

run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END    # short physics check when validating

# Data arrays for the post-run matplotlib plots (initialized before the loop).
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Cache handles fetched once and reused every step (avoid repeated getter calls).
motor_h = motor              # cache: motor handle reused every step
slider_h = slider            # cache: slider handle reused every step

data_file = None
motion_file = None
try:
    # guard file opens: disk full / permission errors surface here, not mid-loop
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk / permission failure on open
        print(f"Failed to open output CSV: {exc}")
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow(["time", "crank_angle", "slider_pos_x", "slider_speed_x"])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "body", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            angle = motor_h.GetMotorAngle()             # integrated crank angle [rad]
            spos = slider_h.GetPos()                     # slider position (cached body)
            svel = slider_h.GetPosDt()                   # slider velocity
            array_time.append(t)
            array_angle.append(angle)
            array_pos.append(spos.x)
            array_speed.append(svel.x)
            data_writer.writerow([t, angle, spos.x, svel.x])
            motion_writer.writerow([t, "slider", spos.x, spos.y, spos.z, svel.x, svel.y, svel.z])
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    print(f"Simulation aborted: {exc}")
    raise
finally:
    # flush + close any open writers so partial output survives an early failure
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot slider position & speed vs crank angle (pi-based ticks)
angle_arr = np.asarray(array_angle)
pos_arr = np.asarray(array_pos)
speed_arr = np.asarray(array_speed)

fig, (ax1, ax2) = plt.subplots(2, sharex=True, figsize=(8, 6))

ax1.plot(angle_arr, pos_arr, "b-")
ax1.set(ylabel="position [m]", title="Slider position vs crank angle")
ax1.grid(True)

ax2.plot(angle_arr, speed_arr, "r--")
ax2.set(ylabel="speed [m/s]", xlabel="crank angle [rad]", title="Slider speed vs crank angle")
ax2.grid(True)

# Tick the crank-angle axis in multiples of pi (0, pi/2, pi, 3pi/2, 2pi).
ax2.set_xticks(np.linspace(0.0, 2.0 * np.pi, 5))
ax2.set_xticklabels(["0", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"])
if angle_arr.size > 0:
    ax2.set_xlim(0.0, min(angle_arr.max(), 2.0 * np.pi))

fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=120)
plt.close(fig)

print(f"Done. Logged {len(array_time)} steps to {SIM_END if not HEADLESS else run_end:.2f} s.")
