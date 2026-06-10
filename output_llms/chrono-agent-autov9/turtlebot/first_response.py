"""TurtleBot differential-drive robot on rigid terrain (PyChrono 9.0.1, Irrlicht).

Model
-----
A Chrono NSC multibody system (ChSystemNSC, Bullet collision) with Earth gravity
contains a fixed rigid ground box (the terrain) and a pychrono.robot.TurtleBot
two-wheel differential-drive robot. The robot is spawned just above the ground so
its two active drive wheels (left=LD=0, right=RD=1) and its caster/passive wheels
rest on the support; the relation between the ground-top height and the robot
base height is asserted at build time so the wheels start on (not through) the
terrain.

Behavior / objective
---------------------
The robot is commanded through its wheel motors to perform scripted maneuvers:
it first drives straight forward (both drive wheels spun the same way), then at a
scheduled time executes a LEFT turn (left wheel held, right wheel driven), and at
a later scheduled time executes a RIGHT turn (right wheel held, left wheel driven).
The robot base pose and velocity are logged every physics step; the expected
behavior is a moving base whose planar heading changes during the two turn phases.

System type: NSC (non-smooth contact). Main bodies: rigid ground box + TurtleBot
(chassis_body plus plates, rods and four wheels created by the TurtleBot model).
"""

# === Imports ===
import os
import math
import csv

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the timeseries PNG
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / schedule) ===
TIME_STEP = 2.0e-3          # s, contact-stable step for this rigid robot
SIM_END = 6.0               # s, total simulated time
RENDER_FPS = 30.0           # frames per second written for the review video
GRAVITY = -9.81             # m/s^2, world -Z

# Ground box (full extents) and placement.
GROUND_LX, GROUND_LY, GROUND_LZ = 20.0, 20.0, 1.0   # m, full extents
GROUND_DENSITY = 1000.0                              # kg/m^3 (fixed, mass unused)
GROUND_CENTER_Z = -1.0                               # m, box center
GROUND_TOP_Z = GROUND_CENTER_Z + 0.5 * GROUND_LZ     # precomputed once -> -0.5 m

# TurtleBot spawn. The model's wheels hang ~0.05 m below the base reference, so
# the verified rest relation places the base 0.05 m above the ground top.
ROBOT_BASE_CLEARANCE = 0.05                           # m, base above ground top
ROBOT_SPAWN = chrono.ChVector3d(0.0, 0.0, GROUND_TOP_Z + ROBOT_BASE_CLEARANCE)
ROBOT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)          # identity (no initial yaw)

# Wheel id enum is not exposed to Python: left drive = 0 (LD), right drive = 1 (RD).
WHEEL_LD = 0
WHEEL_RD = 1
DRIVE_SPEED = math.pi        # rad/s magnitude commanded to a driven wheel

# Maneuver schedule (seconds). Forward, then left turn, then right turn.
T_FORWARD = 0.5
T_LEFT_TURN = 2.0
T_RIGHT_TURN = 4.0

CONTACT_FRICTION = 0.8
CONTACT_RESTITUTION = 0.0

# Derived render cadence, precomputed once (never recomputed in the loop).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps / frame

# Headless validation gate: skip the window, run a short bounded physics check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END


def find_body_by_name(system, name):
    """Return the first body in `system` whose name matches `name` (else None)."""
    for body in system.GetBodies():
        if body.GetName() == name:
            return body
    return None


def main():
    # === System & gravity === NSC system with Bullet collision for rigid contact.
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # === Bodies === fixed rigid ground (terrain) the robot drives on.
    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(CONTACT_FRICTION)
    ground_mat.SetRestitution(CONTACT_RESTITUTION)
    ground = chrono.ChBodyEasyBox(
        GROUND_LX, GROUND_LY, GROUND_LZ, GROUND_DENSITY, True, True, ground_mat
    )
    ground.SetPos(chrono.ChVector3d(0, 0, GROUND_CENTER_Z))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # === Robot === TurtleBot differential-drive robot spawned on the terrain.
    bot = robot.TurtleBot(system, ROBOT_SPAWN, ROBOT_ROT)
    bot.Initialize()

    # The chassis ("base") body is created by the model; fetch it once for logging.
    base = find_body_by_name(system, "chassis_body")  # cache: base body, reused every step
    if base is None:
        raise RuntimeError("TurtleBot chassis_body not found after Initialize()")

    # Assert the base starts on (not through) the terrain: above ground top, below
    # one full clearance margin. Guards a bad spawn/height relation.
    assert base.GetPos().z > GROUND_TOP_Z, "robot base spawned below terrain top"
    assert base.GetPos().z < GROUND_TOP_Z + 0.5, "robot base spawned far above terrain"

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("TurtleBot on rigid terrain")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0, 1.8, 0.6),
                      chrono.ChVector3d(0, 0, 0.0))         # eye, target (AFTER Initialize)
        vis.AddTypicalLights()
        vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                               chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output dirs ===
    os.makedirs("frames", exist_ok=True)  # guard against missing frame output dir
    os.makedirs("cam", exist_ok=True)     # guard against missing motion-log dir

    # === Main loop === render-cadence outer loop; physics + CSV in the inner batch.
    data_file = None
    motion_file = None
    times, xs, ys, yaws, speeds = [], [], [], [], []
    left_was_set = right_was_set = False  # so each scheduled command is applied once
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(
            ["time", "base_x", "base_y", "base_z", "yaw_deg",
             "vx", "vy", "vz", "speed", "ld_wheel_w", "rd_wheel_w"]
        )
        motion_writer.writerow(
            ["time", "body", "x", "y", "z", "vx", "vy", "vz", "speed"]
        )

        frame = 0
        commanded_phase = -1  # tracks which schedule branch is active (for clarity)
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                t = system.GetChTime()

                # --- Scheduled differential-drive commands (each applied once) ---
                if t >= T_FORWARD and commanded_phase < 0:
                    # drive straight forward: both wheels same sense
                    bot.SetMotorSpeed(-DRIVE_SPEED, WHEEL_LD)
                    bot.SetMotorSpeed(-DRIVE_SPEED, WHEEL_RD)
                    commanded_phase = 0
                if t >= T_LEFT_TURN and not left_was_set:
                    # LEFT turn: hold left wheel, drive right wheel
                    bot.SetMotorSpeed(0.0, WHEEL_LD)
                    bot.SetMotorSpeed(-DRIVE_SPEED, WHEEL_RD)
                    left_was_set = True
                    commanded_phase = 1
                if t >= T_RIGHT_TURN and not right_was_set:
                    # RIGHT turn: hold right wheel, drive left wheel
                    bot.SetMotorSpeed(-DRIVE_SPEED, WHEEL_LD)
                    bot.SetMotorSpeed(0.0, WHEEL_RD)
                    right_was_set = True
                    commanded_phase = 2

                # --- Log base pose/velocity (every physics step) ---
                pos = base.GetPos()
                vel = base.GetPosDt()
                rot = base.GetRot()
                yaw_deg = math.degrees(rot.GetCardanAnglesZYX().z)  # heading about world Z
                speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
                ld_w = bot.GetActiveWheelAngVel(WHEEL_LD).y
                rd_w = bot.GetActiveWheelAngVel(WHEEL_RD).y

                data_writer.writerow(
                    [f"{t:.5f}", f"{pos.x:.6f}", f"{pos.y:.6f}", f"{pos.z:.6f}",
                     f"{yaw_deg:.4f}", f"{vel.x:.6f}", f"{vel.y:.6f}", f"{vel.z:.6f}",
                     f"{speed:.6f}", f"{ld_w:.6f}", f"{rd_w:.6f}"]
                )
                motion_writer.writerow(
                    [f"{t:.5f}", "turtlebot_base", f"{pos.x:.6f}", f"{pos.y:.6f}",
                     f"{pos.z:.6f}", f"{vel.x:.6f}", f"{vel.y:.6f}", f"{vel.z:.6f}",
                     f"{speed:.6f}"]
                )
                times.append(t)
                xs.append(pos.x)
                ys.append(pos.y)
                yaws.append(yaw_deg)
                speeds.append(speed)

                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:                    # disk / permission on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot logged base motion vs time.
    if times:
        t_arr = np.array(times)
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(t_arr, np.array(xs), label="base x")
        axes[0].plot(t_arr, np.array(ys), label="base y")
        axes[0].set_ylabel("position [m]")
        axes[0].legend(); axes[0].grid(True)
        axes[1].plot(t_arr, np.array(yaws), color="tab:green", label="yaw")
        axes[1].set_ylabel("yaw [deg]")
        axes[1].legend(); axes[1].grid(True)
        axes[2].plot(t_arr, np.array(speeds), color="tab:red", label="speed")
        axes[2].set_ylabel("speed [m/s]")
        axes[2].set_xlabel("time [s]")
        axes[2].legend(); axes[2].grid(True)
        fig.suptitle("TurtleBot base motion (forward / left turn / right turn)")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
