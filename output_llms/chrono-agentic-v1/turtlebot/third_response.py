"""
TurtleBot differential-drive simulation with lidar sensor and randomly placed boxes.

System type: ChSystemNSC with Bullet collision.
Main bodies: fixed ground plane (center at z=-0.6), TurtleBot robot, 5 randomly placed obstacle boxes.
Sensors: ChLidarSensor mounted on the TurtleBot chassis for environment scanning.
Expected behavior: TurtleBot drives straight while the lidar scans the surroundings;
    the robot translates forward, sweeping the lidar point cloud across the obstacle field.
"""

import os
import math
import random
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
TIME_STEP    = 2e-3          # TurtleBot recommended time step (s)
SIM_END      = 10.0          # simulation duration (s)
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Ground position: as specified (center of 1 m thick box -> top surface at z = -0.6 + 0.5 = -0.1)
GROUND_POS = chrono.ChVector3d(0, 0, -0.6)

# TurtleBot wheel motor speed (rad/s) — forward drive
WHEEL_SPEED_FWD = -math.pi   # negative = forward for TurtleBot

LEFT_WHEEL  = 0   # WheelID: 0 = LEFT
RIGHT_WHEEL = 1   # WheelID: 1 = RIGHT

# Random box parameters
NUM_BOXES     = 5
BOX_HALF_SIZE = 0.2          # half-extent per dimension (full cube = 0.4 m)
BOX_MASS      = 5.0          # kg
BOX_SPREAD    = 3.5          # place boxes in ±BOX_SPREAD XY around origin
RANDOM_SEED   = 42           # reproducible placement

# Lidar sensor parameters
LIDAR_UPDATE_RATE    = 5.0        # Hz
LIDAR_H_SAMPLES      = 800
LIDAR_V_SAMPLES      = 300
LIDAR_H_FOV          = 2.0 * chrono.CH_PI
LIDAR_MAX_VERT_ANGLE = chrono.CH_PI / 12
LIDAR_MIN_VERT_ANGLE = -chrono.CH_PI / 6
LIDAR_MAX_RANGE      = 100.0
LIDAR_COLLECT_WINDOW = 1.0 / LIDAR_UPDATE_RATE   # precomputed once

# === System & gravity ===
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
ground.SetPos(GROUND_POS)
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Randomly placed obstacle boxes ===
random.seed(RANDOM_SEED)
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.1)

# Ground top surface is at z = GROUND_POS.z + 0.5 = -0.1
# Box bottom rests at ground top, so box center = -0.1 + BOX_HALF_SIZE = 0.1
GROUND_TOP_Z = GROUND_POS.z + 0.5         # precomputed once
BOX_CENTER_Z = GROUND_TOP_Z + BOX_HALF_SIZE  # precomputed once

boxes = []
for i in range(NUM_BOXES):
    bx = random.uniform(-BOX_SPREAD, BOX_SPREAD)
    by = random.uniform(-BOX_SPREAD, BOX_SPREAD)
    box = chrono.ChBodyEasyBox(
        2 * BOX_HALF_SIZE, 2 * BOX_HALF_SIZE, 2 * BOX_HALF_SIZE,
        BOX_MASS, True, True, box_mat
    )
    box.SetPos(chrono.ChVector3d(bx, by, BOX_CENTER_Z))
    system.Add(box)
    boxes.append(box)

# === TurtleBot robot ===
# Spawn robot so chassis sits above ground top (-0.1); set z = 0.2 for clearance.
tb_pos = chrono.ChVector3d(0, 0, 0.2)
tb_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, tb_pos, tb_rot)
robot_tb.Initialize()   # no-arg for TurtleBot

# Find chassis body by name (TurtleBot has no GetChassis() API).
# cache: fetched once, reused for sensor mounting and per-step logging
chassis_body = next(
    (b for b in system.GetBodies() if b.GetName() == "chassis_body"),
    None,
)
if chassis_body is None:
    raise RuntimeError("TurtleBot chassis_body not found in system after Initialize().")

# === Motion control function ===
def move(mode: str) -> None:
    """Control TurtleBot wheel motors for differential drive.

    Args:
        mode: 'straight' — both wheels forward at equal speed;
              'left'     — pivot left (right wheel drives, left wheel stopped);
              'right'    — pivot right (left wheel drives, right wheel stopped).

    Raises:
        ValueError: if mode is not one of the three valid strings.
    """
    if mode == 'straight':
        robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD, LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD, RIGHT_WHEEL)
    elif mode == 'left':
        robot_tb.SetMotorSpeed(0.0,              LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD,  RIGHT_WHEEL)
    elif mode == 'right':
        robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD,  LEFT_WHEEL)
        robot_tb.SetMotorSpeed(0.0,              RIGHT_WHEEL)
    else:
        raise ValueError(f"Unknown move mode: {mode!r}. Use 'straight', 'left', or 'right'.")

# === Sensor manager ===
manager = sens.ChSensorManager(system)
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

# === Lidar sensor (mounted on chassis) ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.4),   # above chassis center
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    LIDAR_UPDATE_RATE,
    lidar_offset,
    LIDAR_H_SAMPLES,
    LIDAR_V_SAMPLES,
    LIDAR_H_FOV,
    LIDAR_MAX_VERT_ANGLE,
    LIDAR_MIN_VERT_ANGLE,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,           # sample radius
    0.003,       # vertical divergence angle
    0.003,       # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(LIDAR_COLLECT_WINDOW)

# Lidar filter chain (order matters)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot with Lidar Sensor and Obstacle Boxes")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -4, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.09), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Recording setup (review-only) ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            move('straight')     # command TurtleBot to drive forward
            manager.Update()     # update lidar and all sensors
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # CSV is managed inside the review-only block; nothing to clean up here
