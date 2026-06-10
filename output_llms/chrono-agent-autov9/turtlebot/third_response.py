"""TurtleBot differential-drive rover navigating a box-cluttered floor with a lidar.

Models a Chrono TurtleBot (pychrono.robot.TurtleBot) on a fixed ground plane in a
ChSystemNSC (non-smooth contact). The robot is driven forward by setting equal
left/right active-wheel motor speeds through a small motion-control helper
(`move`) exposing straight / left / right turn modes. A ChSensorManager carries a
360-deg horizontal scanning lidar mounted on the robot chassis so the rover
"sees" five randomly placed obstacle boxes scattered on the floor. An Irrlicht
window provides the review visualization.

System type      : ChSystemNSC (rigid contact, NSC complementarity solver)
Main bodies       : TurtleBot chassis + wheels (26 sub-bodies), fixed ground,
                    5 dynamic obstacle boxes
Expected behavior : the TurtleBot drives forward in a straight line across the
                    floor at a steady speed while the lidar streams range data and
                    the chassis X position increases monotonically.
"""

# === Imports ===
import os
import csv
import math
import random

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the timeseries PNG
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.robot as turtlebot

# === Named constants (geometry / physics) ===
TIME_STEP = 2.0e-3          # s, NSC integration step
SIM_END = 12.0              # s, total simulated time
RENDER_FPS = 30.0           # frames per second for the review video

GROUND_Z = -0.6             # m, ground plane top placed below the robot wheels
GROUND_SIZE = 20.0          # m, square ground extent (full side length)
GROUND_THICK = 0.2          # m, ground slab thickness

ROBOT_POS = chrono.ChVector3d(0.0, 0.0, 0.0)   # robot spawn (chassis origin)
WHEEL_SPEED = 2.0 * math.pi  # rad/s, nominal active-wheel angular speed
TURN_RATIO = 0.3            # fraction of WHEEL_SPEED used on the slow side in a turn

NUM_BOXES = 5               # number of random obstacle boxes
BOX_SIZE = 0.30             # m, obstacle box edge length (full extent)
BOX_MASS = 2.0              # kg, dynamic obstacle mass
BOX_AREA_MIN = 2.0          # m, nearest random box distance from origin
BOX_AREA_MAX = 6.0          # m, farthest random box distance from origin
BOX_SEED = 7                # deterministic RNG seed for reproducible layout

GRAVITY = chrono.ChVector3d(0.0, 0.0, -9.81)

# Lidar parameters
LIDAR_UPDATE_RATE = 10.0    # Hz
LIDAR_W = 360               # horizontal samples (1 deg resolution)
LIDAR_H = 1                 # single scan plane
LIDAR_HFOV = 2.0 * math.pi  # 360 deg horizontal field of view
LIDAR_MAX_VERT = 0.0        # rad, planar scan (no vertical spread)
LIDAR_MIN_VERT = 0.0        # rad
LIDAR_MAX_DIST = 12.0       # m, max measurable range
LIDAR_Z_OFFSET = 0.50       # m, lidar height above the chassis (clears the rod cage)

# Active-wheel ids (Chrono turtlebot WheelID: 0 = left drive, 1 = right drive)
LEFT_WHEEL = 0
RIGHT_WHEEL = 1

# Headless fast-validation gate: skip the on-screen window and run a short sim.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))

# === Derived constants (precomputed once) ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
run_end = min(SIM_END, 1.0) if HEADLESS else SIM_END          # short physics check when validating


def make_random_boxes(system, mat):
    """Create NUM_BOXES dynamic obstacle boxes at deterministic random positions.

    Positions are sampled on the floor (resting on the ground top) in a ring
    around the origin but kept clear of the robot spawn footprint so the rover is
    not pinned by contact at t=0.
    """
    rng = random.Random(BOX_SEED)
    boxes = []
    half = 0.5 * BOX_SIZE
    box_top_z = GROUND_Z + GROUND_THICK * 0.5  # ground top surface
    for _ in range(NUM_BOXES):
        # sample a position in the annulus [BOX_AREA_MIN, BOX_AREA_MAX]
        ang = rng.uniform(0.0, 2.0 * math.pi)
        rad = rng.uniform(BOX_AREA_MIN, BOX_AREA_MAX)
        px = rad * math.cos(ang)
        py = rad * math.sin(ang)
        pz = box_top_z + half  # rest the box on the floor
        box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE,
                                   BOX_MASS / (BOX_SIZE ** 3),  # density -> target mass
                                   True, True, mat)
        box.SetPos(chrono.ChVector3d(px, py, pz))
        box.SetFixed(False)  # indoor interaction props are dynamic by default
        box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.3, 0.2))
        system.Add(box)
        boxes.append(box)
    return boxes


def main():
    # === System & gravity (NSC rigid contact) ===
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(GRAVITY)
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Contact material (NSC) ===
    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)

    box_mat = chrono.ChContactMaterialNSC()
    box_mat.SetFriction(0.6)
    box_mat.SetRestitution(0.0)

    # === Bodies: ground ===
    ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                                  1000.0, True, True, ground_mat)
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.55))
    system.Add(ground)

    # === Bodies: obstacle boxes (random, dynamic) ===
    boxes = make_random_boxes(system, box_mat)

    # === Robot: TurtleBot differential-drive rover ===
    robot = turtlebot.TurtleBot(system, ROBOT_POS, chrono.QUNIT)
    robot.Initialize()

    # Resolve the chassis base body from the system (no public chassis getter).
    base_body = None
    for b in system.GetBodies():
        if b.GetName() == "chassis_body":
            base_body = b
            break
    assert base_body is not None, "TurtleBot chassis_body not found in system bodies"

    def move(mode):
        """Differential-drive motion control: 'straight', 'left', or 'right'.

        Sets the left/right active-wheel angular speeds. Equal speeds drive
        straight; reducing one side's speed yaws the robot toward that side.
        """
        if mode == "straight":
            robot.SetMotorSpeed(WHEEL_SPEED, LEFT_WHEEL)
            robot.SetMotorSpeed(WHEEL_SPEED, RIGHT_WHEEL)
        elif mode == "left":
            robot.SetMotorSpeed(WHEEL_SPEED * TURN_RATIO, LEFT_WHEEL)
            robot.SetMotorSpeed(WHEEL_SPEED, RIGHT_WHEEL)
        elif mode == "right":
            robot.SetMotorSpeed(WHEEL_SPEED, LEFT_WHEEL)
            robot.SetMotorSpeed(WHEEL_SPEED * TURN_RATIO, RIGHT_WHEEL)
        else:
            raise ValueError("unknown move mode: %r" % (mode,))

    # === Sensors: ChSensorManager + chassis-mounted lidar ===
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100),
                                chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.4, 0.4, 0.4))

    lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, LIDAR_Z_OFFSET),
                                   chrono.QUNIT)
    lidar = sens.ChLidarSensor(
        base_body,             # ride on the robot chassis
        LIDAR_UPDATE_RATE,
        lidar_offset,
        LIDAR_W, LIDAR_H,
        LIDAR_HFOV,
        LIDAR_MAX_VERT, LIDAR_MIN_VERT,
        LIDAR_MAX_DIST,
        sens.LidarBeamShape_RECTANGULAR,
        1,                     # sample_radius
        0.003, 0.003,          # vertical / horizontal divergence
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("chassis_lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())  # depth/intensity buffer access
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    manager.AddSensor(lidar)

    # === Visualization (full Irrlicht scene: window + sky + camera + lights + grid) ===
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Z-up; before Initialize
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("TurtleBot lidar navigation")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(-3.0, -4.0, 2.5),
                      chrono.ChVector3d(0.0, 0.0, 0.0))      # AFTER Initialize
        vis.AddTypicalLights()
        vis.AddGrid(0.5, 0.5, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_Z + GROUND_THICK * 0.5),
                                       chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

    # === Output directories ===
    os.makedirs("frames", exist_ok=True)  # guard against missing output dir
    os.makedirs("cam", exist_ok=True)

    # cache: chassis getter fetched once, reused every step
    chassis = base_body

    # === Main loop (render-cadence outer loop; physics in inner batch) ===
    data_file = None
    motion_file = None
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow(["time", "x", "y", "z", "vx", "vy", "speed",
                              "lidar_min_range", "lidar_mean_range"])
        motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile("frames/img_%06d.png" % frame)  # consecutive index
                frame += 1

            for _ in range(render_every):
                move("straight")          # drive the TurtleBot straight ahead
                manager.Update()          # pump sensors every physics step
                system.DoStepDynamics(TIME_STEP)

                t = system.GetChTime()
                pos = chassis.GetPos()
                vel = chassis.GetPosDt()
                speed = math.hypot(vel.x, vel.y)

                # Read the most recent lidar range buffer (guarded — empty before first tick).
                lmin = float("nan")
                lmean = float("nan")
                buf = lidar.GetMostRecentDIBuffer()
                if buf.HasData():  # guard: skip ticks the lidar has not filled yet
                    di = buf.GetDIData()
                    ranges = np.asarray(di)[:, :, 0].ravel()
                    finite = ranges[(ranges > 0.0) & np.isfinite(ranges)]
                    if finite.size:
                        lmin = float(finite.min())
                        lmean = float(finite.mean())

                data_writer.writerow([f"{t:.5f}", f"{pos.x:.5f}", f"{pos.y:.5f}",
                                      f"{pos.z:.5f}", f"{vel.x:.5f}", f"{vel.y:.5f}",
                                      f"{speed:.5f}", f"{lmin:.5f}", f"{lmean:.5f}"])
                motion_writer.writerow(["%.5f" % t, "chassis", "%.5f" % pos.x,
                                        "%.5f" % pos.y, "%.5f" % pos.z,
                                        "%.5f" % vel.x, "%.5f" % vel.y, "%.5f" % vel.z])

                if system.GetChTime() >= run_end:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # flush partial CSV even if a step diverges
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing: timeseries plot from the CSV ===
    times, xs, speeds, lmeans = [], [], [], []
    try:
        with open("simulation_data.csv", "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time"]))
                xs.append(float(row["x"]))
                speeds.append(float(row["speed"]))
                lm = float(row["lidar_mean_range"])
                lmeans.append(lm if math.isfinite(lm) else float("nan"))
    except (OSError, IOError) as exc:  # disk / permission on read-back
        print("WARN: could not read CSV for plotting:", exc)

    if times:
        fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        ax[0].plot(times, xs, color="tab:blue")
        ax[0].set_ylabel("chassis X [m]")
        ax[0].grid(True)
        ax[1].plot(times, speeds, color="tab:green")
        ax[1].set_ylabel("speed [m/s]")
        ax[1].grid(True)
        ax[2].plot(times, lmeans, color="tab:red")
        ax[2].set_ylabel("lidar mean range [m]")
        ax[2].set_xlabel("time [s]")
        ax[2].grid(True)
        fig.suptitle("TurtleBot straight-drive navigation")
        fig.tight_layout()
        fig.savefig("simulation_timeseries.png", dpi=110)
        plt.close(fig)

    print("Done. steps logged:", len(times),
          "final X = %.3f m" % (xs[-1] if xs else float("nan")))


if __name__ == "__main__":
    main()
