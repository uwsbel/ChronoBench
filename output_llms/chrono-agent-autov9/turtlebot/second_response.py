"""TurtleBot differential-drive locomotion on a flat ground patch (PyChrono 9.0.1, Irrlicht).

Model
-----
A pychrono.robot.TurtleBot two-wheel differential-drive robot is spawned above a
large fixed ground patch and driven through three scripted maneuvers by commanding
its two active wheel motor speeds:
  * straight  : both wheels forward at the same speed
  * left      : left wheel slower / reversed, right wheel forward (turn left)
  * right     : right wheel slower / reversed, left wheel forward (turn right)

System type
-----------
ChSystemNSC (non-smooth contact), Bullet collision, Z-up gravity. The ground is a
fixed collision box with an NSC contact material; the robot wheels carry their own
contact material so friction drives the chassis.

Main bodies
-----------
  * ground          : large fixed box, top surface at z = GROUND_TOP_Z
  * TurtleBot        : multi-body robot (chassis + plates + 2 active + 2 passive
                       wheels). The base body is resolved by name ("chassis_body")
                       from the system body list (no public chassis getter exists).

Expected behavior
-----------------
The robot drives forward in a straight line for the first STRAIGHT_END seconds,
then curves left until LEFT_END seconds, then curves right for the remainder. The
logged base pose traces a straight segment followed by two opposite-curvature arcs;
the base stays upright and on the ground (no fall-through, no divergence).
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants: geometry, physics, timeline ===
TIME_STEP = 2.0e-3                 # solver step [s]
SIM_END = 15.0                     # total simulated time [s]
RENDER_FPS = 30.0                  # review-video frame cadence [frames/s]

GRAVITY = -9.81                    # gravitational acceleration along +Z [m/s^2]

# Ground patch (large fixed box). Its TOP face is the drive surface.
GROUND_CENTER_Z = -0.6             # ground box center Z (per requested ground position)
GROUND_SX = 40.0                   # ground extent X [m]
GROUND_SY = 40.0                   # ground extent Y [m]
GROUND_SZ = 2.0                    # ground box thickness [m]
GROUND_TOP_Z = GROUND_CENTER_Z + 0.5 * GROUND_SZ   # precomputed once: top surface Z = 0.4

GROUND_FRICTION = 0.8              # ground/wheel friction coefficient
GROUND_RESTITUTION = 0.0          # no bounce

# Robot spawn: place the chassis-frame origin above the ground top so the wheels
# settle onto the surface (TurtleBot wheel contact sits ~0.2 m below the init Z).
ROBOT_SPAWN_CLEARANCE = 0.22       # init-Z above ground top so wheels touch, not clip
ROBOT_SPAWN_Z = GROUND_TOP_Z + ROBOT_SPAWN_CLEARANCE   # precomputed once

# Differential-drive motor speeds [rad/s] (positive = forward roll). Both wheels
# stay forward in a turn (the inner wheel just slower) so the light chassis sweeps
# a smooth arc instead of spinning in place; tuned for visible, stable motion.
WHEEL_SPEED_FWD = 12.0             # both wheels for straight driving
WHEEL_SPEED_TURN_OUTER = 12.0      # outer (faster) wheel speed during a turn
WHEEL_SPEED_TURN_INNER = 6.0       # inner (slower) wheel speed during a turn (still forward)

# Brief settle before driving so the chassis drops onto its wheels from the spawn
# clearance and the first drive command acts on a robot already resting on ground.
SETTLE_END = 0.4                   # zero-speed settle window [s] at the very start

# Active-wheel ids: 0 = left, 1 = right (TurtleBot WheelID; bare ints accepted).
WHEEL_ID_LEFT = 0
WHEEL_ID_RIGHT = 1

# Maneuver timeline.
STRAIGHT_END = 5.0                 # straight driving for t in [0, STRAIGHT_END)
LEFT_END = 10.0                    # left turn for t in [STRAIGHT_END, LEFT_END)
# right turn for t >= LEFT_END

# Output paths (relative to this script's directory).
FRAMES_DIR = "frames"
CAM_DIR = "cam"
DATA_CSV = "simulation_data.csv"
MOTION_CSV = os.path.join(CAM_DIR, "motion_log.csv")
PLOT_PNG = "simulation_timeseries.png"

# Fast, windowless validation run (short, no Irrlicht window) when set.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# === System & gravity: NSC + Bullet collision, Z-up ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies: ground patch + TurtleBot ===
# Ground contact material (NSC system -> NSC material).
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)

ground = chrono.ChBodyEasyBox(GROUND_SX, GROUND_SY, GROUND_SZ, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_CENTER_Z))
ground.SetFixed(True)
ground_vis = chrono.ChVisualMaterial()
ground_vis.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.GetVisualShape(0).SetMaterial(0, ground_vis)
sys.Add(ground)

# Wheel contact material so wheel/ground friction can propel the chassis.
wheel_mat = chrono.ChContactMaterialNSC()
wheel_mat.SetFriction(GROUND_FRICTION)
wheel_mat.SetRestitution(GROUND_RESTITUTION)

# TurtleBot(system, init_pos, init_rot, wheel_mat). Initialize() builds all sub-bodies.
robot_pos = chrono.ChVector3d(0, 0, ROBOT_SPAWN_Z)
robot_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: facing +X
turtlebot = robot.TurtleBot(sys, robot_pos, robot_rot, wheel_mat)
turtlebot.Initialize()

# Resolve the base body by name — TurtleBot exposes no public chassis getter.
base_body = None
for body in sys.GetBodies():                    # cache: scan once, keep handle for loop
    if body.GetName() == "chassis_body":
        base_body = body
        break
assert base_body is not None, "could not resolve TurtleBot chassis_body from system bodies"
# Sanity: chassis must start above the ground top, not embedded in it.
assert base_body.GetPos().z > GROUND_TOP_Z, "robot spawned below ground top — would clip"


# === Drive control: scripted differential-drive maneuvers ===
def move(mode):
    """Set the two active-wheel motor speeds for a named drive mode.

    mode: "settle" | "straight" | "left" | "right". Raises ValueError otherwise.
    """
    if mode == "settle":
        turtlebot.SetMotorSpeed(0.0, WHEEL_ID_LEFT)
        turtlebot.SetMotorSpeed(0.0, WHEEL_ID_RIGHT)
    elif mode == "straight":
        turtlebot.SetMotorSpeed(WHEEL_SPEED_FWD, WHEEL_ID_LEFT)
        turtlebot.SetMotorSpeed(WHEEL_SPEED_FWD, WHEEL_ID_RIGHT)
    elif mode == "left":
        # outer (left) wheel faster, inner (right) wheel slower -> sweep a left arc
        # (verified by the logged yaw rate sign for this robot's drive direction)
        turtlebot.SetMotorSpeed(WHEEL_SPEED_TURN_OUTER, WHEEL_ID_LEFT)
        turtlebot.SetMotorSpeed(WHEEL_SPEED_TURN_INNER, WHEEL_ID_RIGHT)
    elif mode == "right":
        # outer (right) wheel faster, inner (left) wheel slower -> sweep a right arc
        turtlebot.SetMotorSpeed(WHEEL_SPEED_TURN_INNER, WHEEL_ID_LEFT)
        turtlebot.SetMotorSpeed(WHEEL_SPEED_TURN_OUTER, WHEEL_ID_RIGHT)
    else:
        raise ValueError("invalid move mode: {!r} (expected settle/straight/left/right)".format(mode))


def mode_for_time(t):
    """Map simulation time to the scripted maneuver."""
    if t < SETTLE_END:
        return "settle"
    if t < STRAIGHT_END:
        return "straight"
    if t < LEFT_END:
        return "left"
    return "right"


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("TurtleBot differential-drive maneuvers")
    vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.5, -3.0, 2.0), chrono.ChVector3d(0, 0, GROUND_TOP_Z))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.001), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Main loop === render-cadence outer loop; physics + CSV logging in inner batch
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

os.makedirs(FRAMES_DIR, exist_ok=True)   # guard against missing frames dir
os.makedirs(CAM_DIR, exist_ok=True)      # guard against missing cam dir

data_f = None
motion_f = None
last_mode = None
try:
    # open with context managers so writers always flush/close
    with open(DATA_CSV, "w", newline="") as data_f, open(MOTION_CSV, "w", newline="") as motion_f:
        data_w = csv.writer(data_f)
        motion_w = csv.writer(motion_f)
        data_w.writerow(["time", "mode", "x", "y", "z", "vx", "vy", "vz", "speed"])
        motion_w.writerow(["time", "body", "x", "y", "z", "qe0", "qe1", "qe2", "qe3",
                           "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(os.path.join(FRAMES_DIR, "img_%06d.png" % frame))
                frame += 1
            for _ in range(render_every):
                t = sys.GetChTime()
                mode = mode_for_time(t)
                if mode != last_mode:            # only re-command on transitions
                    move(mode)
                    print("t=%.2f s -> robot action: %s" % (t, mode))
                    last_mode = mode

                pos = base_body.GetPos()
                vel = base_body.GetPosDt()
                rot = base_body.GetRot()
                speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
                data_w.writerow([t, mode, pos.x, pos.y, pos.z,
                                 vel.x, vel.y, vel.z, speed])
                motion_w.writerow([t, "chassis_body", pos.x, pos.y, pos.z,
                                   rot.e0, rot.e1, rot.e2, rot.e3, vel.x, vel.y, vel.z])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
except (OSError, IOError) as exc:                 # disk / permission on CSV or frame write
    import traceback
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:         # solver divergence / bad drive mode
    import traceback
    traceback.print_exc()
    raise
finally:
    # CSV files are closed by the with-block; report what was produced.
    print("Simulation finished at t=%.3f s" % sys.GetChTime())

# === Post-processing: timeseries plot from the logged CSV ===
try:
    times, xs, ys, speeds = [], [], [], []
    with open(DATA_CSV, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            speeds.append(float(row["speed"]))

    if times:
        t_arr = np.array(times)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.plot(xs, ys, "-b")
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.set_title("TurtleBot base ground track")
        ax1.axis("equal")
        ax1.grid(True)

        ax2.plot(t_arr, speeds, "-r")
        ax2.axvline(STRAIGHT_END, color="g", ls="--", label="straight->left")
        ax2.axvline(LEFT_END, color="m", ls="--", label="left->right")
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("base speed [m/s]")
        ax2.set_title("Base speed vs time")
        ax2.legend()
        ax2.grid(True)

        fig.tight_layout()
        fig.savefig(PLOT_PNG, dpi=110)
        plt.close(fig)
        print("Wrote %s (%d samples)" % (PLOT_PNG, len(times)))
except (OSError, IOError) as exc:                 # plot/file write failure is non-fatal
    import traceback
    traceback.print_exc()
