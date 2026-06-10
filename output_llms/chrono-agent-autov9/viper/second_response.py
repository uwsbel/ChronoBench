"""Viper lunar rover driving on a rigid ground patch, with an onboard third-person
camera sensor (OptiX) riding the chassis.

System type: ChSystemNSC (rigid-body, non-smooth contact).
Main bodies:
  - The NASA VIPER rover (pychrono.robot.Viper): chassis, four steerable wheels,
    suspension arms/uprights, driven by a DC-motor speed controller.
  - A large fixed ground patch (ChBodyEasyBox) with a contact material that the
    wheels roll on.
Sensors:
  - An OptiX ChCameraSensor attached to the rover chassis giving a third-person
    point-of-view (offset 1.0 m ahead, 1.45 m above, pitched 0.2 rad about +Y),
    rendered at 720x480 with a 1.408 rad horizontal FOV. Frames are saved to
    cam/pov/ for assembly into a review video.
Expected behavior:
  The rover drives forward under a constant no-load wheel speed; the chassis
  translates across the patch with a near-constant forward velocity while the
  onboard camera tracks the scene from behind/above the rover. The chassis pose
  and velocity are logged each step; the trajectory should advance monotonically
  in the driving direction with no NaN / blow-up.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless plotting backend
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.robot as viper_robot

# === Named constants (geometry / physics) ===
TIME_STEP = 1.0e-3                  # solver step (s)
SIM_END = 8.0                       # simulated duration (s)
RENDER_FPS = 25.0                   # review-frame cadence (Hz)

GRAVITY = chrono.ChVector3d(0, 0, -9.81)   # Z-up world

GROUND_SIZE_X = 40.0                # ground patch extents (m)
GROUND_SIZE_Y = 40.0
GROUND_THICKNESS = 1.0
GROUND_TOP_Z = 0.0                  # top surface of ground at world z = 0

ROVER_START_Z = 0.5                 # chassis spawn height so wheels start on (not through) ground
ROVER_START = chrono.ChVector3d(-5.0, 0.0, ROVER_START_Z)

WHEEL_NO_LOAD_SPEED = math.pi       # rad/s commanded to the DC motor (forward drive)
STEER_ANGLE = 0.0                   # straight-line drive (rad)

GROUND_FRICTION = 0.9
GROUND_RESTITUTION = 0.0

# Camera-sensor parameters (third-person POV riding the chassis).
CAM_UPDATE_RATE = 15.0              # Hz
CAM_WIDTH = 720
CAM_HEIGHT = 480
CAM_FOV = 1.408                     # horizontal field of view (rad)
CAM_OFFSET = chrono.ChVector3d(1.0, 0, 1.45)            # ahead + above the chassis origin
CAM_OFFSET_ROT = chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))  # slight downward pitch

# Derived constants (precomputed once).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: physics steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))          # fast windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating

# === System & gravity === build the NSC system the rover and ground share.
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(GRAVITY)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies: ground patch === fixed support the wheels roll on (needs contact + collision).
ground_mat = chrono.ChContactMaterialNSC()   # NSC material to match ChSystemNSC
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)

ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_THICKNESS,
    1000.0,        # density (irrelevant; fixed body)
    True,          # visualize
    True,          # collide
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_TOP_Z - GROUND_THICKNESS / 2.0))  # top surface at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.62))
system.Add(ground)

# === Bodies: VIPER rover === chassis + 4 steerable driven wheels + suspension.
rover = viper_robot.Viper(system)
driver = viper_robot.ViperDCMotorControl()                 # DC-motor speed controller
driver.SetMotorNoLoadSpeed(WHEEL_NO_LOAD_SPEED, viper_robot.V_LF)
driver.SetMotorNoLoadSpeed(WHEEL_NO_LOAD_SPEED, viper_robot.V_RF)
driver.SetMotorNoLoadSpeed(WHEEL_NO_LOAD_SPEED, viper_robot.V_LB)
driver.SetMotorNoLoadSpeed(WHEEL_NO_LOAD_SPEED, viper_robot.V_RB)
driver.SetSteering(STEER_ANGLE)                            # straight-line drive
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_START, chrono.QUNIT))

# cache: fetched once, reused every step (chassis body + its initial pose)
chassis = rover.GetChassis()
start_pos = rover.GetChassisPos()
assert start_pos.z > GROUND_TOP_Z, "chassis must spawn above the ground surface"

# === Sensor manager & lighting === required for the OptiX camera sensor.
manager = sens.ChSensorManager(system)
LIGHT_INTENSITY = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(LIGHT_INTENSITY, LIGHT_INTENSITY, LIGHT_INTENSITY),
    500.0,
)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # ambient fill for the sensor scene

# === Sensor: third-person POV camera === rides the chassis body.
offset_pose = chrono.ChFramed(CAM_OFFSET, CAM_OFFSET_ROT)
cam = sens.ChCameraSensor(
    chassis.GetBody(),     # attach to the rover chassis so the view follows the rover
    CAM_UPDATE_RATE,
    offset_pose,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Third Person POV")
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "Viper POV Camera"))
cam.PushFilter(sens.ChFilterSave("cam/pov/"))     # PNG frames -> mp4 by RUN stage
cam.PushFilter(sens.ChFilterRGBA8Access())        # frame-buffer access
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
vis = None
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("VIPER Rover with Onboard POV Camera")
    vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-8, -6, 4), chrono.ChVector3d(-5, 0, 0.5))  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                chrono.ChColor(0.35, 0.35, 0.35))       # ground reference grid

# === Main loop === render-cadence outer loop; physics + sensors in the inner batch.
os.makedirs("frames", exist_ok=True)   # guard against missing output dir for review frames
os.makedirs("cam", exist_ok=True)      # guard against missing dir for motion log + sensor frames

data_file = None
motion_file = None
try:
    # Open both CSV writers via context managers so they always flush/close.
    with open("simulation_data.csv", "w", newline="") as data_file, \
         open("cam/motion_log.csv", "w", newline="") as motion_file:
        data_writer = csv.writer(data_file)
        data_writer.writerow(
            ["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "speed"]
        )
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(
            ["time", "body", "pos_x", "pos_y", "pos_z",
             "rot_w", "rot_x", "rot_y", "rot_z", "vel_x", "vel_y", "vel_z"]
        )

        times, speeds, xs = [], [], []   # for the post-run plot

        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                rover.Update()            # advance the rover controller/actuators each step
                manager.Update()          # pump the camera sensor every physics step

                t = system.GetChTime()
                pos = rover.GetChassisPos()
                vel = rover.GetChassisVel()
                rot = rover.GetChassisRot()
                speed = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

                data_writer.writerow([t, pos.x, pos.y, pos.z,
                                      vel.x, vel.y, vel.z, speed])
                motion_writer.writerow([t, "viper_chassis", pos.x, pos.y, pos.z,
                                        rot.e0, rot.e1, rot.e2, rot.e3,
                                        vel.x, vel.y, vel.z])
                times.append(t)
                speeds.append(speed)
                xs.append(pos.x)

                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= RUN_END:
                    break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Context managers above already flush/close the CSV writers on the way out.
    print(f"[viper] simulated to t={system.GetChTime():.3f}s")

# === Post-processing === plot logged trajectory + speed vs time.
if times:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, xs, label="chassis x")
    ax1.set_ylabel("x position (m)")
    ax1.grid(True)
    ax1.legend()
    ax2.plot(times, speeds, color="tab:orange", label="chassis speed")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("speed (m/s)")
    ax2.grid(True)
    ax2.legend()
    fig.suptitle("VIPER rover chassis motion")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=120)
    plt.close(fig)

    arr = np.array(speeds)
    print(f"[viper] frames={frame if not HEADLESS else 0} steps={len(times)} "
          f"x: {xs[0]:.3f} -> {xs[-1]:.3f} m | "
          f"speed mean={arr.mean():.4f} max={arr.max():.4f} m/s | "
          f"NaN={np.isnan(arr).any()}")
