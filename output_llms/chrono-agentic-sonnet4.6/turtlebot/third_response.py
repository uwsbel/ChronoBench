"""
TurtleBot differential-drive simulation on rigid terrain with lidar sensor and random boxes.

System type: ChSystemNSC (rovers use NSC)
Bodies: fixed ground plane, TurtleBot (chassis + 2 drive wheels built by robot.TurtleBot),
        5 randomly placed static boxes as obstacles.
Sensors: ChLidarSensor mounted on TurtleBot chassis for environment scanning.
Expected behavior: TurtleBot drives straight continuously using the move() control function,
                   lidar scans the environment, 5 box obstacles are randomly distributed.
"""

import math
import os
import random
import csv

import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
TIME_STEP    = 2e-3    # TurtleBot uses 2e-3 s
SIM_END      = 15.0    # simulation duration (s)
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# TurtleBot wheel IDs
LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1

# Ground: 1 m thick box; top surface at z = -0.6 + 0.5 = -0.1
GROUND_Z = -0.6

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground body ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(40, 40, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Random box obstacles (5 boxes, deterministic seed) ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.1)

random.seed(42)
BOX_POSITIONS = [
    chrono.ChVector3d(random.uniform(1.5, 4.0), random.uniform(-2.0, 2.0), 0.15)
    for _ in range(5)
]
BOX_SIZE = 0.3  # full width of each box (m)

for box_pos in BOX_POSITIONS:
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, 500, True, True, box_mat)
    box.SetPos(box_pos)
    box.SetFixed(False)  # dynamic — can be pushed by the robot
    system.Add(box)

# === TurtleBot rover ===
init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w, x, y, z)

robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()

# TurtleBot does not expose GetChassis(); look up chassis by name in the system
chassis_body = next(
    b for b in system.GetBodies() if b.GetName() == "chassis_body"
)  # cache: fetched once, reused for sensors and camera tracking

# === Motion control function ===
def move(mode):
    """Control TurtleBot movement via differential motor speeds.

    Args:
        mode: 'straight' — both wheels forward at equal speed
              'left'     — pivot left (right wheel only)
              'right'    — pivot right (left wheel only)
    """
    speed = -math.pi  # rad/s forward (negative = forward in TurtleBot convention)
    if mode == 'straight':
        robot_tb.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot_tb.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# === Sensor manager & lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# Lidar mounted on TurtleBot chassis (2D horizontal scan, 360° FOV)
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.25),   # offset above chassis center
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)),
)
h_samples = 800
v_samples = 1    # 2D lidar: single horizontal scan line

lidar = sens.ChLidarSensor(
    chassis_body,              # attach to TurtleBot chassis
    5.0,                       # update_rate (Hz)
    lidar_offset,              # offset pose on chassis
    h_samples,                 # horizontal samples
    v_samples,                 # vertical samples (1 = 2D)
    2 * chrono.CH_PI,          # horizontal FOV (full 360°)
    0,                         # max_vert_angle (0 for 2D)
    0,                         # min_vert_angle (0 for 2D)
    100.0,                     # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                         # sample_radius
    0.003,                     # vert divergence_angle
    0.003,                     # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)   # collection window = 1 / update_rate

# Lidar filter chain (order matters)
lidar.PushFilter(sens.ChFilterVisualize(h_samples, v_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot with Lidar Sensor and Random Boxes")
vis.Initialize()   # Initialize FIRST — then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-2, -3, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# Camera chase offset — follow the robot from behind and above
CAM_OFFSET = chrono.ChVector3d(-1.5, 0, 1.5)   # behind and above chassis


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        # Update Irrlicht camera to follow TurtleBot
        tb_pos = chassis_body.GetPos()    # cache: reused for camera + CSV
        cam_eye    = tb_pos + CAM_OFFSET
        cam_target = tb_pos
        vis.UpdateCamera(cam_eye, cam_target)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()

            # Drive TurtleBot straight using motion control function
            move('straight')

            # Update sensor manager — exactly once per physics step
            manager.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
