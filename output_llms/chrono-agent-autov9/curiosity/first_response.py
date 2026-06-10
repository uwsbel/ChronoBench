"""Curiosity Mars rover navigating on a flat rigid terrain (PyChrono 9.0.1).

Model
-----
* System  : ChSystemNSC (non-smooth contact), gravity -9.81 m/s^2 along world Z.
* Terrain : a single large fixed box body ("ground") with an NSC contact
            material and collision enabled, acting as the rigid driving surface.
* Robot   : pychrono.robot.Curiosity, the six-wheeled NASA Curiosity rover, built
            on the shared system and driven by a CuriosityDCMotorControl driver
            (DC-motor speed control on every drive wheel) that also receives a
            real-time steering setpoint.

Expected behavior
-----------------
The rover spawns just above the ground, settles onto its six wheels, and drives
forward under DC-motor control. A small steering setpoint makes it follow a
gently curving path, so the chassis position (logged to CSV) advances steadily
in X/Y while the chassis Z stays close to the wheel-contact height.

Visualization uses Irrlicht (window, sky box, logo, explicit camera, typical
lights, and a ground reference grid). Frames are written to frames/ for an
offline review video; chassis pose/velocity is logged to CSV and plotted.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# === Named constants (geometry / physics) ===
TIME_STEP = 2.0e-3            # solver step [s]
SIM_END = 12.0               # simulated duration [s]
RENDER_FPS = 30.0            # review-video frame rate
GRAVITY_Z = -9.81            # gravity along world Z [m/s^2]

GROUND_SIZE_X = 40.0         # ground patch full extent X [m]
GROUND_SIZE_Y = 40.0         # ground patch full extent Y [m]
GROUND_THICKNESS = 1.0       # ground box thickness [m]
GROUND_TOP_Z = 0.0           # ground top surface height [m]

ROVER_SPAWN_Z = 0.18         # chassis spawn height above ground top [m] (small settle)
# Fixed elevated camera eye in world coords: the rover visibly translates across
# the frame against the static grid, while the look-at target tracks the rover so
# it stays in view. (A camera rigidly chasing the rover hides its translation.)
CAMERA_EYE = chrono.ChVector3d(-4.0, -6.0, 4.0)
DRIVE_NO_LOAD_SPEED = math.pi      # DC drive-motor no-load speed [rad/s]
DRIVE_STALL_TORQUE = 300.0         # DC drive-motor stall torque [N*m]
STEER_ANGLE = 0.18                 # constant steering setpoint [rad]

GROUND_FRICTION = 0.9        # terrain contact friction
GROUND_RESTITUTION = 0.0     # terrain restitution (no bounce)

# Derived placement (precomputed once)
GROUND_CENTER_Z = GROUND_TOP_Z - 0.5 * GROUND_THICKNESS   # box center so top sits at GROUND_TOP_Z
ROVER_INIT_POS = chrono.ChVector3d(0.0, 0.0, GROUND_TOP_Z + ROVER_SPAWN_Z)

# Headless validation gate: fast windowless physics check (no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END      # short check when validating

OUT_CSV = "simulation_data.csv"
MOTION_CSV = "cam/motion_log.csv"
PLOT_PNG = "simulation_timeseries.png"

# === System & gravity === NSC system for hard rover/terrain contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY_Z))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === fixed rigid ground box with NSC contact material + collision
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)

ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_THICKNESS,
    1000.0,          # density (irrelevant — body is fixed)
    True,            # visualization
    True,            # collision
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_CENTER_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === Rover === Curiosity built on the shared system + DC-motor driver
rover = robot.Curiosity(sys)

driver = robot.CuriosityDCMotorControl()
# Configure every drive wheel's DC motor (no-load speed + stall torque).
WHEEL_IDS = [robot.C_LF, robot.C_RF, robot.C_LM, robot.C_RM, robot.C_LB, robot.C_RB]
for wid in WHEEL_IDS:
    driver.SetMotorNoLoadSpeed(DRIVE_NO_LOAD_SPEED, wid)
    driver.SetMotorStallTorque(DRIVE_STALL_TORQUE, wid)
driver.SetSteering(STEER_ANGLE)   # real-time steering setpoint (radians)

rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, chrono.QUNIT))

# Footprint sanity: the rover spawns at the patch center, well inside the ground
# AABB — assert it sits above the surface so it settles onto (not through) it.
assert ROVER_INIT_POS.z > GROUND_TOP_Z, "rover must spawn above the ground surface"
assert abs(ROVER_INIT_POS.x) < 0.5 * GROUND_SIZE_X, "rover X outside ground patch"

# === Visualization === full Irrlicht scene: window + sky + logo + camera + lights + grid
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
vis = None
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity rover on rigid terrain")
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # gravity along -Z
    vis.Initialize()                                     # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()                                      # outdoor sky backdrop
    vis.AddCamera(CAMERA_EYE,                             # fixed eye (AFTER Initialize)
                  chrono.ChVector3d(0.0, 0.0, 0.3))      # look at the rover start
    vis.AddTypicalLights()                               # standard lighting
    vis.AddLightWithShadow(chrono.ChVector3d(12.0, -12.0, 18.0),
                           chrono.ChVector3d(0.0, 0.0, 0.0),
                           45, 20, 55, 35, 512)           # shadow-casting light
    vis.AddGrid(1.0, 1.0, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.001), chrono.QUNIT),
                chrono.ChColor(0.35, 0.35, 0.35))         # ground reference grid
    vis.EnableShadows()

# === Main loop === render-cadence outer loop; rover.Update() + physics each step
os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)      # review video / motion-log dir

data_f = None
motion_f = None
times, xs, ys, zs, speeds = [], [], [], [], []

try:
    try:
        data_f = open(OUT_CSV, "w", newline="")
        motion_f = open(MOTION_CSV, "w", newline="")
    except (OSError, IOError) as exc:        # disk full / permission denied
        print(f"[error] could not open CSV output: {exc}")
        raise

    data_w = csv.writer(data_f)
    motion_w = csv.writer(motion_f)
    data_w.writerow(["time", "pos_x", "pos_y", "pos_z",
                     "vel_x", "vel_y", "vel_z", "speed", "rover_mass"])
    motion_w.writerow(["time", "body", "pos_x", "pos_y", "pos_z",
                       "vel_x", "vel_y", "vel_z"])

    rover_mass = rover.GetRoverMass()   # cache: constant, fetched once

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            # Fixed eye, target tracks the rover: it visibly drives across the frame
            # against the static grid while staying centered in view.
            cam_target = rover.GetChassisPos()
            vis.UpdateCamera(CAMERA_EYE, cam_target)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index
            frame += 1

        for _ in range(render_every):
            rover.Update()                       # advance the rover controller each step
            sys.DoStepDynamics(TIME_STEP)

            t = sys.GetChTime()
            pos = rover.GetChassisPos()          # base pose
            vel = rover.GetChassisVel()          # base linear velocity
            speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

            data_w.writerow([t, pos.x, pos.y, pos.z,
                             vel.x, vel.y, vel.z, speed, rover_mass])
            motion_w.writerow([t, "curiosity_chassis",
                               pos.x, pos.y, pos.z, vel.x, vel.y, vel.z])
            times.append(t); xs.append(pos.x); ys.append(pos.y)
            zs.append(pos.z); speeds.append(speed)

            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close writers so partial output survives even on a mid-run error.
    if data_f is not None:
        data_f.close()
    if motion_f is not None:
        motion_f.close()

# === Post-processing === plot logged chassis trajectory / speed vs time
if times:
    t_arr = np.array(times)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(t_arr, np.array(xs), label="pos_x")
    ax1.plot(t_arr, np.array(ys), label="pos_y")
    ax1.plot(t_arr, np.array(zs), label="pos_z")
    ax1.set_ylabel("chassis position [m]")
    ax1.legend(loc="best"); ax1.grid(True)
    ax2.plot(t_arr, np.array(speeds), color="tab:red", label="speed")
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("chassis speed [m/s]")
    ax2.legend(loc="best"); ax2.grid(True)
    fig.suptitle("Curiosity rover on rigid terrain")
    fig.tight_layout()
    with open(PLOT_PNG, "wb") as pf:    # context-managed PNG write
        fig.savefig(pf, dpi=110)
    plt.close(fig)

    final_x, final_y = xs[-1], ys[-1]
    horiz = math.sqrt(final_x * final_x + final_y * final_y)
    print(f"[done] steps logged={len(times)} final_pos=({final_x:.3f},{final_y:.3f},{zs[-1]:.3f}) "
          f"horizontal_travel={horiz:.3f} m max_speed={max(speeds):.3f} m/s")
else:
    print("[warn] no data logged")
