"""TurtleBot differential-drive scene with boxes and a lidar sensor.

This NSC/Bullet simulation places the built-in PyChrono TurtleBot on a rigid
ground box whose center is at z=-0.6, adds five randomly placed dynamic boxes,
and drives the TurtleBot straight ahead while a 2D lidar scans the scene.
"""

import math
import random
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants ===
TIME_STEP = 2.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
GROUND_SIZE_X = 12.0
GROUND_SIZE_Y = 8.0
GROUND_SIZE_Z = 1.0
GROUND_CENTER_Z = -0.6
GROUND_TOP_Z = GROUND_CENTER_Z + 0.5 * GROUND_SIZE_Z
TURTLEBOT_START = chrono.ChVector3d(-3.0, 0.0, 0.0)
TURTLEBOT_ROT = chrono.QUNIT
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
STRAIGHT_WHEEL_SPEED = -8.0 * math.pi
BOX_COUNT = 5
BOX_SIZE = 0.45
BOX_DENSITY = 450.0
LIDAR_RATE = 5.0
LIDAR_H_SAMPLES = 720
LIDAR_V_SAMPLES = 1


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(80)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Bodies ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.85)
ground_mat.SetRestitution(0.05)

ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_SIZE_Z, 1000.0, True, True, ground_mat
)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_CENTER_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.65)
box_mat.SetRestitution(0.1)

random.seed(42)
boxes = []
for box_index in range(BOX_COUNT):
    x_pos = random.uniform(-1.0, 3.6)
    y_pos = random.uniform(-2.0, 2.0)
    box = chrono.ChBodyEasyBox(
        BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, box_mat
    )
    box.SetPos(chrono.ChVector3d(x_pos, y_pos, GROUND_TOP_Z + 0.5 * BOX_SIZE))
    system.Add(box)
    boxes.append(box)

robot_tb = robot.TurtleBot(system, TURTLEBOT_START, TURTLEBOT_ROT)
robot_tb.Initialize()

chassis_body = next(
    body for body in system.GetBodies() if body.GetName() == "chassis_body"
)  # cache: TurtleBot chassis body found once after Initialize


# === Motion control ===
def move(mode):
    """Command TurtleBot wheel speeds for straight, left, and right motion."""
    if mode == "straight":
        left_speed = STRAIGHT_WHEEL_SPEED
        right_speed = STRAIGHT_WHEEL_SPEED
    elif mode == "left":
        left_speed = 0.0
        right_speed = STRAIGHT_WHEEL_SPEED
    elif mode == "right":
        left_speed = STRAIGHT_WHEEL_SPEED
        right_speed = 0.0
    else:
        raise ValueError(f"unknown TurtleBot motion mode: {mode}")

    robot_tb.SetMotorSpeed(left_speed, LEFT_DRIVE_WHEEL)
    robot_tb.SetMotorSpeed(right_speed, RIGHT_DRIVE_WHEEL)
    return left_speed, right_speed


# === Sensor manager & lidar ===
manager = sens.ChSensorManager(system)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.45),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    LIDAR_RATE,
    lidar_offset,
    LIDAR_H_SAMPLES,
    LIDAR_V_SAMPLES,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    15.0,
    sens.LidarBeamShape_RECTANGULAR,
    1,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("TurtleBot 2D Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "TurtleBot Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot with 2D lidar and boxes")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5.0, -6.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512,
)
vis.AddGrid(
    0.5,
    0.5,
    24,
    16,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.005), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop ===
tracked_chassis = chassis_body  # cache: reused every step for review logging
frame = 0


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            left_cmd, right_cmd = move("straight")
            manager.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
