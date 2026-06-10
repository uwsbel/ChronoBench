"""
TurtleBot differential-drive simulation with lidar sensor and obstacle boxes.

System: ChSystemNSC with Bullet collision.
Main bodies: fixed ground plane, TurtleBot robot (2-wheel differential drive),
             5 randomly placed box obstacles.
Sensors: ChLidarSensor (2D lidar) mounted on the TurtleBot chassis.
Expected behavior: TurtleBot drives straight using the move() motion-control
function while the lidar sensor scans the environment and the sensor manager
updates each step.
"""

import math
import os
import random

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens

# === Named constants ===
TIME_STEP   = 2e-3          # TurtleBot uses 2 ms steps
SIM_END     = 15.0          # total simulation duration (s)
RENDER_FPS  = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

GROUND_THICKNESS = 1.0
GROUND_Z         = -0.6     # ground box center; top surface at z = -0.6 + 0.5 = -0.1
GROUND_HALF      = GROUND_THICKNESS / 2.0

# TurtleBot spawn (above ground top surface)
TB_SPAWN_Z  = 0.2
TB_INIT_POS = chrono.ChVector3d(0, 0, TB_SPAWN_Z)
TB_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# Wheel IDs
LEFT_WHEEL  = 0
RIGHT_WHEEL = 1
WHEEL_SPEED = -math.pi   # forward rad/s

# Box obstacle parameters
BOX_COUNT     = 5
BOX_HALF      = 0.15     # half-size of each cube side
BOX_MASS      = 2.0
BOX_Z         = -0.1 + BOX_HALF   # rest on ground top surface (ground top at -0.1)

# Lidar parameters
LIDAR_UPDATE_RATE   = 5.0          # Hz
LIDAR_H_SAMPLES     = 800
LIDAR_V_SAMPLES     = 1            # 2-D lidar
LIDAR_H_FOV         = 2 * chrono.CH_PI
LIDAR_MAX_RANGE     = 100.0
LIDAR_SAMPLE_RADIUS = 2
LIDAR_DIV_ANGLE     = 0.003

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, GROUND_THICKNESS, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === TurtleBot robot ===
robot_tb = robot.TurtleBot(sys, TB_INIT_POS, TB_INIT_ROT)
robot_tb.Initialize()

chassis_body = robot_tb.GetChassis().GetBody()  # cache: fetched once, reused for sensor mount

# === Motion control ===
def move(mode: str) -> None:
    """Control TurtleBot differential drive.

    Modes:
      'straight' — both wheels at equal forward speed.
      'left'     — pivot left (right wheel only).
      'right'    — pivot right (left wheel only).
    """
    if mode == "straight":
        robot_tb.SetMotorSpeed(WHEEL_SPEED, LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED, RIGHT_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0.0,         LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED, RIGHT_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(WHEEL_SPEED, LEFT_WHEEL)
        robot_tb.SetMotorSpeed(0.0,         RIGHT_WHEEL)
    else:
        raise ValueError(f"Unknown move mode: {mode!r}")

# === Box obstacles (randomly placed, dynamic) ===
random.seed(42)   # reproducible layout
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.1)
boxes = []
for _ in range(BOX_COUNT):
    bx = random.uniform(-4.0, 4.0)
    by = random.uniform(-4.0, 4.0)
    # keep boxes away from the robot's spawn
    while math.hypot(bx, by) < 1.0:
        bx = random.uniform(-4.0, 4.0)
        by = random.uniform(-4.0, 4.0)
    box = chrono.ChBodyEasyBox(
        BOX_HALF * 2, BOX_HALF * 2, BOX_HALF * 2,
        BOX_MASS, True, True, box_mat,
    )
    box.SetPos(chrono.ChVector3d(bx, by, BOX_Z))
    sys.Add(box)
    boxes.append(box)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar sensor mounted on chassis, looking forward/horizontally
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.3),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    LIDAR_UPDATE_RATE,
    lidar_offset,
    LIDAR_H_SAMPLES,
    LIDAR_V_SAMPLES,
    LIDAR_H_FOV,
    0.0,                                  # max_vert_angle (2-D: 0)
    0.0,                                  # min_vert_angle (2-D: 0)
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_RADIUS,
    LIDAR_DIV_ANGLE,
    LIDAR_DIV_ANGLE,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot — lidar + boxes")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3.0, 2.0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            move("straight")
            manager.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:     # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
