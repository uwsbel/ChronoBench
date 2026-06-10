"""TurtleBot rigid-body rover simulation with randomly placed box obstacles and a chassis-mounted lidar.

The script builds a Z-up ChSystemNSC scene with Bullet collision, a fixed box ground
at the requested vertical position, five deterministic obstacle boxes, a built-in
PyChrono TurtleBot driven straight by differential wheel speeds, an Irrlicht window,
and a lidar sensor managed by ChSensorManager. The expected behavior is forward
TurtleBot motion while the lidar scans the nearby boxes.
"""

import math
import random
import traceback

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants === named dimensions and rates keep the robot/scene reproducible
TIME_STEP = 2.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
WHEEL_SPEED = -8.0 * math.pi
GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_SIZE_Z = 1.0
GROUND_Z = -0.6
GROUND_TOP_Z = GROUND_Z + 0.5 * GROUND_SIZE_Z
BOX_SIZE = 0.35
BOX_DENSITY = 500.0
BOX_COUNT = 5
RANDOM_SEED = 7
TURTLEBOT_START = chrono.ChVector3d(0.0, 0.2, 0.0)
TURTLEBOT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)


# === System & gravity === NSC rover contacts require Bullet collision
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(80)


# === Bodies === ground, obstacle boxes, and the built-in TurtleBot model share one system
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_SIZE_Z, 1000, True, True, ground_mat)
ground.SetName("ground")
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.AddBody(ground)

box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.75)
box_mat.SetRestitution(0.05)
random.seed(RANDOM_SEED)
boxes = []
candidate_positions = [
    (1.1, -0.8),
    (1.8, 0.7),
    (2.6, -1.1),
    (3.2, 0.9),
    (4.0, -0.3),
]
for box_index in range(BOX_COUNT):
    base_x, base_y = candidate_positions[box_index]
    jitter_x = random.uniform(-0.12, 0.12)
    jitter_y = random.uniform(-0.12, 0.12)
    obstacle = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_DENSITY, True, True, box_mat)
    obstacle.SetName(f"random_box_{box_index}")
    obstacle.SetPos(chrono.ChVector3d(base_x + jitter_x, base_y + jitter_y, GROUND_TOP_Z + 0.5 * BOX_SIZE))
    obstacle.SetFixed(False)
    obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.25 + 0.1 * box_index, 0.2))
    system.AddBody(obstacle)
    boxes.append(obstacle)

robot_tb = robot.TurtleBot(system, TURTLEBOT_START, TURTLEBOT_ROT)
robot_tb.Initialize()
chassis_body = system.SearchBody("chassis_body")  # cache: chassis body reused for sensors and logging
if chassis_body is None:
    raise RuntimeError("TurtleBot chassis body was not created")


# === Controls === differential wheel commands expose straight, left, and right motion modes
def move(mode):
    if mode == "straight":
        robot_tb.SetMotorSpeed(WHEEL_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(WHEEL_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"unsupported TurtleBot motion mode: {mode}")


# === Sensor manager === lidar rides on the TurtleBot chassis and scans the obstacle field
manager = sens.ChSensorManager(system)
lidar_update_rate = 5.0
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.30),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    lidar_update_rate,
    lidar_offset,
    800,
    1,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    25.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("TurtleBot Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization === Irrlicht is initialized before adding sky, camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot lidar with random boxes")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-1.5, -2.2, 1.4), chrono.ChVector3d(1.6, 0.2, 0.1))
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
    1.0,
    1.0,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render review frames while each physics step advances controls and sensors
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        robot_pos = chassis_body.GetPos()
        vis.UpdateCamera(robot_pos + chrono.ChVector3d(-1.0, -1.6, 0.9), robot_pos + chrono.ChVector3d(0.8, 0.0, 0.1))
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            move("straight")
            manager.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError, OSError) as exc:  # solver/sensor/runtime guards surface concrete failures
    traceback.print_exc()
    raise
finally:
    pass
