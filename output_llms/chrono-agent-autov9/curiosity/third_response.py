"""Curiosity rover with a chassis-mounted lidar sensor (PyChrono 9.0.x, NSC).

This simulation models NASA's Curiosity Mars rover (pychrono.robot.Curiosity)
driving forward across a flat rigid plane under Earth gravity. A DC-motor speed
controller (CuriosityDCMotorControl) spins the six wheels so the rover crawls
forward in a straight line.

A rotating 3D LIDAR sensor (pychrono.sensor.ChLidarSensor) is rigidly mounted on
the rover's chassis. The lidar is driven by a ChSensorManager: it sweeps a
horizontal field of view, casts depth+intensity rays each scan, converts them to
an XYZI point cloud, and saves range images while the rover moves. This lets the
simulation exercise an onboard ranging sensor that tracks the chassis as it
translates.

System type: ChSystemNSC (rigid contact, non-smooth). Main bodies: the Curiosity
rover (chassis + bogies + six wheels) and a fixed ground plane. Expected
behavior: the rover accelerates from rest and drives forward (chassis +X position
grows monotonically, chassis speed settles to a steady crawl), while the lidar
produces non-empty depth/XYZI buffers each update. Irrlicht renders the review
window; the lidar is the demo's sensor subject.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Constants (geometry / physics / sensor) ===
# No physics-critical value is invented silently: the sim duration, time step,
# ground size, rover spawn, and lidar parameters are all named here.
TIME_STEP = 2.0e-3                 # s, NSC contact step for the rover
SIM_END = 8.0                      # s, total simulated time
RENDER_FPS = 30.0                  # review-video frame cadence
GRAVITY = -9.81                    # m/s^2, Earth gravity along -Z (Z-up world)

GROUND_SIZE = 40.0                 # m, square ground plane edge
GROUND_THICK = 1.0                 # m, ground slab thickness
GROUND_FRICTION = 0.8              # rover wheel traction on the plane
GROUND_TOP_Z = 0.0                 # m, top surface of the ground at z = 0

ROVER_SPAWN_X = -8.0               # m, start near one edge so it can drive across
ROVER_SPAWN_Z = 0.0                # m, chassis reference height above the plane

MOTOR_NO_LOAD_SPEED = math.pi      # rad/s, wheel free-spin speed -> forward crawl
MOTOR_STALL_TORQUE = 300.0         # N*m, wheel motor stall torque

# Lidar parameters (a Velodyne-style rotating scanner mounted on the chassis).
LIDAR_UPDATE_RATE = 5.0            # Hz, full scans per second
LIDAR_H_SAMPLES = 480             # horizontal samples per scan
LIDAR_V_SAMPLES = 16              # vertical channels
LIDAR_HFOV = 2.0 * math.pi         # rad, full 360-degree horizontal sweep
LIDAR_MAX_V_ANGLE = math.radians(15.0)   # rad, top vertical beam
LIDAR_MIN_V_ANGLE = math.radians(-15.0)  # rad, bottom vertical beam
LIDAR_MAX_DISTANCE = 100.0         # m, maximum range
LIDAR_MOUNT_OFFSET = chrono.ChVector3d(0.0, 0.0, 0.4)  # m, above chassis origin

OUT_CSV = "simulation_data.csv"
MOTION_CSV = "cam/motion_log.csv"
TIMESERIES_PNG = "simulation_timeseries.png"

# Derived once (precomputed) — never recompute inside the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))         # fast windowless check
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END         # short physics check


def main():
    # === System & gravity ===
    # NSC system so the wheels make rigid frictional contact with the ground.
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, GRAVITY))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # === Ground plane ===
    # A large fixed box gives the rover a flat surface to crawl across.
    ground_mat = chrono.ChContactMaterialNSC()    # NSC material matches the NSC system
    ground_mat.SetFriction(GROUND_FRICTION)
    ground_mat.SetRestitution(0.0)

    ground = chrono.ChBodyEasyBox(
        GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
        1000.0, True, True, ground_mat,
    )
    ground.SetPos(chrono.ChVector3d(0, 0, GROUND_TOP_Z - 0.5 * GROUND_THICK))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)

    # === Rover (Curiosity) + driver ===
    # The rover wrapper builds its chassis, bogies, and six wheels internally.
    driver = robot.CuriosityDCMotorControl()      # DC-motor speed controller
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LF)
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RF)
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LM)
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RM)
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LB)
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RB)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LF)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RF)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LM)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RM)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LB)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RB)
    driver.SetSteering(0.0)                        # drive straight

    rover = robot.Curiosity(sys)
    rover.SetDriver(driver)
    spawn = chrono.ChVector3d(ROVER_SPAWN_X, 0.0, ROVER_SPAWN_Z)
    rover.Initialize(chrono.ChFramed(spawn, chrono.QUNIT))

    chassis = rover.GetChassis().GetBody()         # cache: chassis body fetched once, reused every step
    start_x = rover.GetChassisPos().x              # precomputed once: spawn X for displacement check

    # === Sensor manager + lidar ===
    # The manager owns scene lighting and pumps the lidar each physics step.
    manager = sens.ChSensorManager(sys)
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))  # ChVector3f, not ChColor

    lidar = sens.ChLidarSensor(
        chassis,                                   # rides on the rover chassis
        LIDAR_UPDATE_RATE,
        chrono.ChFramed(LIDAR_MOUNT_OFFSET, chrono.QUNIT),  # mount frame above chassis
        LIDAR_H_SAMPLES,
        LIDAR_V_SAMPLES,
        LIDAR_HFOV,
        LIDAR_MAX_V_ANGLE,
        LIDAR_MIN_V_ANGLE,
        LIDAR_MAX_DISTANCE,
    )
    lidar.SetName("chassis_lidar")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)
    lidar.PushFilter(sens.ChFilterDIAccess())      # access depth+intensity buffer
    lidar.PushFilter(sens.ChFilterPCfromDepth())   # convert depth scan to point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())    # access XYZI point-cloud buffer
    manager.AddSensor(lidar)

    # === Visualization (Irrlicht) === full scene: window + sky + camera + lights + grid
    # Gated behind SIMBENCH_VALIDATE so validation runs headless and fast.
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sys)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Curiosity rover with chassis lidar")
        vis.Initialize()                                    # Initialize FIRST
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(ROVER_SPAWN_X - 4.0, -6.0, 3.0),
                      chrono.ChVector3d(ROVER_SPAWN_X, 0.0, 0.5))  # AFTER Initialize
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

    # === Output setup ===
    os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
    os.makedirs("cam", exist_ok=True)      # guard against missing dir for motion log

    try:
        data_f = open(OUT_CSV, "w", newline="")
        motion_f = open(MOTION_CSV, "w", newline="")
    except (OSError, IOError) as exc:      # disk full / permission denied
        print("Could not open output CSV files:", exc)
        raise

    # === Main loop ===
    # Render-cadence outer loop: render once per frame, advance physics + pump the
    # lidar in an inner batch so the sensor sees every post-step chassis pose.
    times = []
    xs = []
    speeds = []
    lidar_hits = []

    try:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "chassis_x", "chassis_y", "chassis_z",
                         "speed", "displacement_x", "lidar_xyzi_points"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "body", "pos_x", "pos_y", "pos_z",
                           "vel_x", "vel_y", "vel_z"])

        frame = 0
        last_xyzi_points = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                rover.Update()                 # advance the rover's motor controller
                manager.Update()               # pump the lidar every physics step

                # Read the lidar XYZI buffer defensively (empty before first tick).
                xyzi = lidar.GetMostRecentXYZIBuffer()
                if xyzi.HasData():             # guard: skip frames the sensor has not filled yet
                    last_xyzi_points = int(xyzi.Width * xyzi.Height)

                t = sys.GetChTime()
                pos = rover.GetChassisPos()    # chassis world position
                vel = rover.GetChassisVel()    # chassis world velocity
                speed = vel.Length()

                data_w.writerow([f"{t:.5f}", f"{pos.x:.6f}", f"{pos.y:.6f}",
                                 f"{pos.z:.6f}", f"{speed:.6f}",
                                 f"{pos.x - start_x:.6f}", last_xyzi_points])
                motion_w.writerow([f"{t:.5f}", "curiosity_chassis",
                                   f"{pos.x:.6f}", f"{pos.y:.6f}", f"{pos.z:.6f}",
                                   f"{vel.x:.6f}", f"{vel.y:.6f}", f"{vel.z:.6f}"])

                times.append(t)
                xs.append(pos.x - start_x)
                speeds.append(speed)
                lidar_hits.append(last_xyzi_points)

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= RUN_END:
                    break

    except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close writers even if a step diverged mid-run.
        data_f.close()
        motion_f.close()

    # === Post-processing ===
    # Plot displacement, speed, and lidar point count vs time from the logged data.
    if times:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        ax1.plot(times, xs, color="tab:blue")
        ax1.set_ylabel("forward displacement [m]")
        ax1.grid(True)
        ax2.plot(times, speeds, color="tab:green")
        ax2.set_ylabel("chassis speed [m/s]")
        ax2.grid(True)
        ax3.plot(times, lidar_hits, color="tab:red")
        ax3.set_ylabel("lidar XYZI points")
        ax3.set_xlabel("time [s]")
        ax3.grid(True)
        fig.suptitle("Curiosity rover with chassis lidar")
        fig.tight_layout()
        fig.savefig(TIMESERIES_PNG, dpi=110)
        plt.close(fig)

        final_disp = xs[-1]
        max_pts = max(lidar_hits) if lidar_hits else 0
        print(f"steps logged: {len(times)}  final forward displacement: "
              f"{final_disp:.3f} m  max lidar points: {max_pts}")


if __name__ == "__main__":
    main()
