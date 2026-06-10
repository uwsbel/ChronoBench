"""Curiosity rover driving forward across a long box obstacle on rigid ground.

Model
-----
A NASA Curiosity (Mars Science Laboratory) rover, built with the PyChrono
`pychrono.robot.Curiosity` model, is placed on a flat rigid ground patch and
commanded to drive straight forward with zero steering. A long, low box
obstacle (a "speed bump" / ridge) lies across the rover's path; the rover
approaches it, climbs over it with its rocker-bogie suspension, and continues.

System type
-----------
`ChSystemNSC` (non-smooth contact, complementarity). Rigid ground plane and a
rigid box obstacle, both given an NSC contact material. The rover wheels are
given a contact material so they can grip the ground and ride over the box.

Main bodies
-----------
- ground          : fixed rigid box acting as the floor
- obstacle        : fixed long rigid box the rover must cross
- Curiosity rover : 6 driven wheels (C_LF/RF, C_LM/RM, C_LB/RB), rocker-bogie
                    suspension, driven by a CuriosityDCMotorControl driver.

Expected behavior
-----------------
The rover starts at x = -5 m, drives in the +X direction with zero steering,
reaches the obstacle, tilts as the front wheels climb it, levels out after the
rear wheels pass, and ends with a clearly larger x than it started. Chassis
pose/velocity are logged each step and plotted at the end.
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
TIME_STEP = 2e-3                      # NSC step; stable for rover + contact
SIM_END = 14.0                        # s; long enough to reach and cross the box
RENDER_FPS = 20.0                     # review-video frame rate

GRAVITY = 9.81                        # m/s^2, world is Z-up

# Ground patch (full extents); thin slab the rover and obstacle rest on.
GROUND_LX, GROUND_LY, GROUND_LZ = 40.0, 10.0, 0.4
GROUND_TOP_Z = 0.0                    # top surface of the ground at z = 0

# Long box obstacle the rover crosses: long across Y (the path width),
# short along X (driving direction), low in Z so the rover can climb it.
OBST_LX, OBST_LY, OBST_LZ = 0.6, 4.0, 0.12
OBST_CENTER_X = 0.0                   # obstacle straddles the world origin
OBST_CENTER_Y = 0.0

# Rover spawn: start behind the obstacle, on the ground, per the request.
ROVER_START_X = -5.0
ROVER_START_Y = 0.0
ROVER_WHEEL_CLEARANCE = 0.2           # lift so wheels start just on the surface

# Contact material parameters (NSC).
GROUND_FRICTION = 0.9
GROUND_RESTITUTION = 0.0

# Driver command: drive straight forward, no steering.
STEERING_ANGLE = 0.0                  # zero steering input -> straight line
MOTOR_NO_LOAD_SPEED = math.pi         # rad/s, wheel free-spin speed
MOTOR_STALL_TORQUE = 300.0            # N*m, wheel stall torque

# === Derived constants (precomputed once) ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
ground_center_z = GROUND_TOP_Z - 0.5 * GROUND_LZ               # precomputed once
obst_center_z = GROUND_TOP_Z + 0.5 * OBST_LZ                   # sits on the ground
rover_start_z = GROUND_TOP_Z + ROVER_WHEEL_CLEARANCE           # precomputed once

# Footprint check: rover spawn must not overlap the obstacle at t=0.
rover_front_x = ROVER_START_X + 1.5   # approx rover half-length ahead of origin
obst_back_x = OBST_CENTER_X - 0.5 * OBST_LX
assert rover_front_x < obst_back_x, "rover spawn overlaps the obstacle footprint"

# Headless validation gate: fast, windowless physics check (no on-screen window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))
run_end = min(SIM_END, 1.0) if HEADLESS else SIM_END   # short physics check when validating

# === System & gravity ===
# NSC system: rigid contact between rover wheels, ground, and the box obstacle.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -GRAVITY))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
# Shared NSC material for ground + obstacle so the wheels get traction.
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)

# === Bodies (ground + obstacle) ===
# Ground: fixed slab; top surface at z = GROUND_TOP_Z.
ground = chrono.ChBodyEasyBox(GROUND_LX, GROUND_LY, GROUND_LZ, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_center_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# Obstacle: a long, low fixed box the rover must climb over.
obstacle = chrono.ChBodyEasyBox(OBST_LX, OBST_LY, OBST_LZ, 1000.0, True, True, ground_mat)
obstacle.SetPos(chrono.ChVector3d(OBST_CENTER_X, OBST_CENTER_Y, obst_center_z))
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.4, 0.2))
sys.Add(obstacle)

# === Rover (Curiosity) + driver ===
# Build the rover on the shared NSC system, give wheels a contact material,
# attach a DC-motor speed controller, and spawn it on the ground behind the box.
rover = robot.Curiosity(sys)
rover.SetWheelContactMaterial(ground_mat)

driver = robot.CuriosityDCMotorControl()
# Apply the DC-motor speed/torque limits to each of the 6 driven wheels.
WHEEL_IDS = [robot.C_LF, robot.C_RF, robot.C_LM, robot.C_RM, robot.C_LB, robot.C_RB]
for wid in WHEEL_IDS:
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, wid)
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, wid)
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(ROVER_START_X, ROVER_START_Y, rover_start_z)
init_rot = chrono.QUNIT                      # facing +X (default), drives forward
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Zero steering -> the rover drives in a straight +X line.
driver.SetSteering(STEERING_ANGLE)

# === Cached handles (fetched once, reused every step) ===
chassis_body = rover.GetChassis().GetBody()   # cache: chassis rigid body, reused every step
rover_mass = rover.GetRoverMass()             # cache: constant rover mass
print("Curiosity rover mass [kg]:", rover_mass)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)        # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity rover crossing a box obstacle")
    vis.Initialize()                                          # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()                                           # outdoor sky backdrop
    vis.AddCamera(chrono.ChVector3d(-7, -6, 3),
                  chrono.ChVector3d(0, 0, 0))                 # AFTER Initialize
    vis.AddTypicalLights()                                    # standard lighting
    vis.AddLight(chrono.ChVector3d(5.0, -8.0, 8.0), 60,
                 chrono.ChColor(0.9, 0.9, 0.9))               # extra fill light
    vis.AddGrid(1.0, 1.0, 40, 20,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))                # ground reference grid

# === Output setup ===
os.makedirs("frames", exist_ok=True)   # guard against missing frame output dir
os.makedirs("cam", exist_ok=True)      # guard against missing motion-log dir

sim_csv = None
motion_csv = None
sim_writer = None
motion_writer = None
times = []
xs = []
zs = []
speeds = []

try:
    sim_csv = open("simulation_data.csv", "w", newline="")          # main physics log
    motion_csv = open("cam/motion_log.csv", "w", newline="")        # body motion contract
    sim_writer = csv.writer(sim_csv)
    motion_writer = csv.writer(motion_csv)
    sim_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z",
                         "vx", "vy", "vz", "speed"])
    motion_writer.writerow(["time", "body", "x", "y", "z",
                            "vx", "vy", "vz"])

    # === Main loop === render-cadence outer loop; physics + logging inner batch
    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        for _ in range(render_every):
            rover.Update()                       # advance the rover controller/motors
            sys.DoStepDynamics(TIME_STEP)

            t = sys.GetChTime()
            pos = chassis_body.GetPos()
            vel = chassis_body.GetPosDt()
            spd = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

            sim_writer.writerow([f"{t:.5f}", f"{pos.x:.6f}", f"{pos.y:.6f}",
                                 f"{pos.z:.6f}", f"{vel.x:.6f}", f"{vel.y:.6f}",
                                 f"{vel.z:.6f}", f"{spd:.6f}"])
            motion_writer.writerow([f"{t:.5f}", "chassis", f"{pos.x:.6f}",
                                    f"{pos.y:.6f}", f"{pos.z:.6f}",
                                    f"{vel.x:.6f}", f"{vel.y:.6f}", f"{vel.z:.6f}"])
            times.append(t)
            xs.append(pos.x)
            zs.append(pos.z)
            speeds.append(spd)

            if sys.GetChTime() >= run_end:
                break

except (RuntimeError, ValueError) as exc:        # solver divergence / bad sim state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:                # disk / permission errors on CSV
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if sim_csv is not None:
        sim_csv.close()
    if motion_csv is not None:
        motion_csv.close()

# === Post-processing === plot chassis trajectory + speed vs time
if times:
    t_arr = np.array(times)
    x_arr = np.array(xs)
    z_arr = np.array(zs)
    s_arr = np.array(speeds)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(t_arr, x_arr, label="chassis x")
    ax1.plot(t_arr, z_arr, label="chassis z")
    ax1.axvline(0, color="0.8")
    ax1.set_ylabel("position [m]")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Curiosity crossing a box obstacle")

    ax2.plot(t_arr, s_arr, color="tab:red", label="speed")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("speed [m/s]")
    ax2.legend(loc="best")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

    print("frames written:", frame if not HEADLESS else 0)
    print("start x:", x_arr[0], "end x:", x_arr[-1],
          "moved:", x_arr[-1] - x_arr[0])
