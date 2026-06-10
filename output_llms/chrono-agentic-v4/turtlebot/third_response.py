"""
TurtleBot simulation with lidar sensor and randomly placed boxes.
plan_type: mbs_in_scene
robot: TurtleBot (differential drive)
system: ChSystemNSC with Bullet collision
"""
import math
import os
import random

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Named constants ===
time_step = 2e-3           # TurtleBot uses 2e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # used in scored loop

LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
STRAIGHT_SPEED = 3.0        # rad/s per wheel (straight forward)


# Ground Z-adjusted: top surface at z=-0.6 per input3
GROUND_TOP_Z = -0.6
GROUND_BOX_Z = GROUND_TOP_Z - 0.5     # 1m thick box -> center at -1.1

# TurtleBot spawn position
init_pos = chrono.ChVector3d(0, 0.2, GROUND_TOP_Z + 0.01)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Random box placement (seeded for reproducibility)
random.seed(42)
BOX_COUNT = 5
BOX_MIN_DIST = 2.0   # min distance from origin (keep clear of TurtleBot spawn)
BOX_MAX_DIST = 5.0   # max distance
BOX_SIZE_MIN = 0.15
BOX_SIZE_MAX = 0.40

# === System & collision ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_BOX_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === TurtleBot ===
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()

# === Sensor mount body (fixed world frame) ===
# TurtleBot does not expose its internal chassis body, so we create a fixed
# mount body at the sensor location for lidar attachment.
sensor_mount = chrono.ChBody()
sensor_mount.SetFixed(True)
sensor_mount.SetPos(chrono.ChVector3d(0, 0, 1.0))  # elevated vantage point
system.Add(sensor_mount)

# === Randomly placed boxes ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.1)
boxes = []
for i in range(BOX_COUNT):
    angle = random.uniform(0, 2 * math.pi)
    dist = random.uniform(BOX_MIN_DIST, BOX_MAX_DIST)
    bx = dist * math.cos(angle)
    by = dist * math.sin(angle)
    bz = GROUND_TOP_Z + 0.1  # sitting on ground

    size_x = random.uniform(BOX_SIZE_MIN, BOX_SIZE_MAX)
    size_y = random.uniform(BOX_SIZE_MIN, BOX_SIZE_MAX)
    size_z = random.uniform(BOX_SIZE_MIN, BOX_SIZE_MAX)

    box = chrono.ChBodyEasyBox(size_x, size_y, size_z, 50.0, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, bz))
    box.SetFixed(False)
    system.Add(box)
    boxes.append(box)

# === Sensor manager + lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# Lidar offset: mounted on chassis, looking forward
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.1, 0, 0.3),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    sensor_mount,
    5.0,                               # update_rate Hz
    lidar_offset,
    800,                               # horizontal_samples
    1,                                 # vertical_samples (2D lidar)
    2 * chrono.CH_PI,                   # horizontal_fov
    0,                                 # max_vert_angle
    0,                                 # min_vert_angle
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Lidar Depth"))
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
vis.SetWindowTitle("TurtleBot - Lidar + Boxes")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Motion control function ===
def move(mode):
    """Control TurtleBot movement: 'straight', 'left', 'right'."""
    if mode == "straight":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# === Main loop ===
sim_time = 0.0
while vis.Run() and sim_time < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        move("straight")
        manager.Update()
        system.DoStepDynamics(time_step)
        sim_time = system.GetChTime()
        if sim_time >= sim_end:
            break
