"""Viper Mars rover driving on a rigid terrain patch (PyChrono 9.0.1 + Irrlicht).

Model
-----
- System: ChSystemNSC (non-smooth contact), gravity along -Z (Z-up world).
- Bodies: a single large fixed rigid ground box (the rigid terrain) carrying a
  NSC contact material, plus the four-wheeled `pychrono.robot.Viper` rover
  (chassis + suspension arms + uprights + four wheels created by the wrapper).
- Actuation: a `ViperDCMotorControl` driver supplies a constant drive speed to
  all wheels; the steering angle is ramped LINEARLY from 0 to a target angle
  over the first part of the run, then held, so the rover follows a curving path.

Expected behavior
------------------
The rover starts at rest on the terrain, begins rolling forward under the
constant wheel speed, and as the steering angle ramps up it curves to one side.
The chassis world position and velocity are logged every physics step; the
chassis should translate (non-zero displacement) and the heading should change
once steering becomes non-zero.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# === Named constants (geometry / physics / control) ===
TIME_STEP = 1.0e-3              # solver step (s)
SIM_END = 14.0                 # total simulated time (s)
RENDER_FPS = 30.0              # review-frame cadence (frames/s)

GRAVITY_Z = -9.81              # m/s^2, Z-up world

# Rigid terrain (a wide, thin, fixed box centered at the origin)
TERRAIN_SIZE_X = 40.0
TERRAIN_SIZE_Y = 40.0
TERRAIN_THICK = 1.0
TERRAIN_TOP_Z = 0.0            # terrain top surface sits at z = 0

# Contact material (terrain + wheels)
FRICTION = 0.9
RESTITUTION = 0.0

# Rover spawn: centered on the terrain, lifted so the wheels rest on the surface
ROVER_SPAWN_X = 0.0
ROVER_SPAWN_Y = 0.0
ROVER_CLEARANCE = 0.30         # base height of chassis above terrain top at spawn

# Control: constant wheel speed + a linearly ramped steering angle
WHEEL_SPEED = math.pi          # rad/s, constant no-load drive speed target
STALL_TORQUE = 300.0           # N*m, motor stall torque
STEER_TARGET = 0.40            # rad, final steering angle (~23 deg)
STEER_RAMP_END = 8.0           # s, time over which steering ramps 0 -> target

# Camera (Irrlicht window viewpoint), look-at the spawn point
CAM_EYE = chrono.ChVector3d(ROVER_SPAWN_X - 3.0, ROVER_SPAWN_Y - 4.5, 2.2)
CAM_TARGET = chrono.ChVector3d(ROVER_SPAWN_X, ROVER_SPAWN_Y, ROVER_CLEARANCE)

# Derived constants (precomputed once — never recomputed inside the loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
ROVER_SPAWN_Z = TERRAIN_TOP_Z + ROVER_CLEARANCE              # precomputed once

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating

OUT_DATA_CSV = "simulation_data.csv"
OUT_MOTION_CSV = os.path.join("cam", "motion_log.csv")
OUT_PLOT = "simulation_timeseries.png"


def steering_angle(t):
    """Linear steering ramp 0 -> STEER_TARGET over [0, STEER_RAMP_END], then hold."""
    if t >= STEER_RAMP_END:
        return STEER_TARGET
    return STEER_TARGET * (t / STEER_RAMP_END)


# === System & gravity === ChSystemNSC with -Z gravity; NSC matches the rigid contact material
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY_Z))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared NSC material for terrain and rover wheels
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(FRICTION)
ground_mat.SetRestitution(RESTITUTION)

# === Bodies === fixed rigid terrain box; its top surface is the rover support plane
ground = chrono.ChBodyEasyBox(
    TERRAIN_SIZE_X, TERRAIN_SIZE_Y, TERRAIN_THICK,
    1000.0,        # density (irrelevant — body is fixed)
    True,          # visualize
    True,          # collide
    ground_mat,
)
# center the box so its TOP face is at TERRAIN_TOP_Z
ground.SetPos(chrono.ChVector3d(0.0, 0.0, TERRAIN_TOP_Z - 0.5 * TERRAIN_THICK))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === Robot (Viper rover) === wrapper builds chassis + 4 suspension arms + 4 wheels
rover = robot.Viper(sys)                       # attaches all rover bodies to `sys`
driver = robot.ViperDCMotorControl()           # DC-motor speed/steering controller
driver.SetMotorNoLoadSpeed(WHEEL_SPEED, robot.V_LF)
driver.SetMotorNoLoadSpeed(WHEEL_SPEED, robot.V_RF)
driver.SetMotorNoLoadSpeed(WHEEL_SPEED, robot.V_LB)
driver.SetMotorNoLoadSpeed(WHEEL_SPEED, robot.V_RB)
driver.SetMotorStallTorque(STALL_TORQUE, robot.V_LF)
driver.SetMotorStallTorque(STALL_TORQUE, robot.V_RF)
driver.SetMotorStallTorque(STALL_TORQUE, robot.V_LB)
driver.SetMotorStallTorque(STALL_TORQUE, robot.V_RB)
rover.SetDriver(driver)
rover.SetWheelContactMaterial(ground_mat)      # wheels share the terrain NSC material

# Spawn the rover upright at the support height (Z-up, identity rotation)
spawn_frame = chrono.ChFramed(
    chrono.ChVector3d(ROVER_SPAWN_X, ROVER_SPAWN_Y, ROVER_SPAWN_Z),
    chrono.QUNIT,
)
rover.Initialize(spawn_frame)

# Sanity: the rover must start ABOVE the terrain top, not clipping through it.
assert ROVER_SPAWN_Z > TERRAIN_TOP_Z, "rover spawned below terrain top"

# Cache handle fetched once and reused every step (avoid repeated getter calls).
# The Viper wrapper exposes chassis pose/velocity directly via GetChassisPos/Vel.
get_pos = rover.GetChassisPos        # cache: bound method, reused every step
get_vel = rover.GetChassisVel        # cache: bound method, reused every step

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Viper rover on rigid terrain")
    vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(CAM_EYE, CAM_TARGET)                  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        chrono.ChVector3d(10.0, 10.0, 12.0),            # light position
        chrono.ChVector3d(0.0, 0.0, 0.0),              # target
        40, 8, 40,                                      # frustum size, near, far
        45, 512,                                        # fov (deg), shadow-map res
    )
    vis.AddGrid(
        1.0, 1.0, 40, 40,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z + 0.01), chrono.QUNIT),
        chrono.ChColor(0.4, 0.4, 0.4),                  # ground reference grid
    )
    vis.EnableShadows()

# === Output setup === guard against a missing output dir before opening writers
os.makedirs("frames", exist_ok=True)   # review-frame PNGs (assembled to mp4 later)
os.makedirs("cam", exist_ok=True)      # motion log + review video live here

data_file = None
motion_file = None
times, pos_x, pos_y, pos_z, speed = [], [], [], [], []

# === Main loop === render-cadence outer loop; physics + logging in the inner batch
try:
    try:
        data_file = open(OUT_DATA_CSV, "w", newline="")          # disk / permission errors
        motion_file = open(OUT_MOTION_CSV, "w", newline="")
    except (OSError, IOError) as exc:                            # cannot open CSV target
        raise RuntimeError("failed to open CSV output files: %s" % exc)

    data_writer = csv.writer(data_file)
    data_writer.writerow(
        ["time", "steer_angle", "pos_x", "pos_y", "pos_z",
         "vel_x", "vel_y", "vel_z", "speed"]
    )
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(
        ["time", "body", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"]
    )

    def log_step(t):
        """Write one CSV row per physics step for the rover chassis pose/velocity."""
        p = get_pos()                     # chassis world position
        v = get_vel()                     # chassis world linear velocity
        spd = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        steer = steering_angle(t)
        data_writer.writerow([t, steer, p.x, p.y, p.z, v.x, v.y, v.z, spd])
        motion_writer.writerow([t, "viper_chassis", p.x, p.y, p.z, v.x, v.y, v.z])
        times.append(t); pos_x.append(p.x); pos_y.append(p.y); pos_z.append(p.z)
        speed.append(spd)

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            # update the steering command (gradual ramp) then advance the rover
            driver.SetSteering(steering_angle(t))
            rover.Update()                # propagate driver commands to the motors
            log_step(t)
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush + close any open writers even if a step diverged mid-run
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot chassis trajectory + speed vs time from the logged arrays
if times:
    t_arr = np.array(times)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.plot(pos_x, pos_y, "-b")
    ax1.set_xlabel("pos_x (m)"); ax1.set_ylabel("pos_y (m)")
    ax1.set_title("Chassis ground-plane trajectory"); ax1.axis("equal"); ax1.grid(True)
    ax2.plot(t_arr, speed, "-r", label="speed")
    ax2.plot(t_arr, np.array(pos_z), "-g", label="pos_z")
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("m/s  /  m")
    ax2.set_title("Chassis speed and height"); ax2.legend(); ax2.grid(True)
    fig.tight_layout()
    fig.savefig(OUT_PLOT, dpi=110)
    plt.close(fig)

print("done: steps logged =", len(times),
      "| final chassis pos = (%.3f, %.3f, %.3f)" % (pos_x[-1], pos_y[-1], pos_z[-1]) if times else "no data")
