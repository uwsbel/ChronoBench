import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

# ------------------------------------------------------------------
# 1) Create the Chrono system
# ------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# ------------------------------------------------------------------
# 2) Create the ground (moved up to z = -0.6)
# ------------------------------------------------------------------
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(
    20, 20, 1,            # x,y,z half‐sizes
    1000,                 # density (not really used for fixed)
    True, True,           # collide?, visualize?
    ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# ------------------------------------------------------------------
# 3) Randomly place 5 small boxes for the robot to interact with
# ------------------------------------------------------------------
for i in range(5):
    box = chrono.ChBodyEasyBox(
        0.5, 0.5, 0.5,      # half‐sizes
        1000,               # density
        True, True)         # collide? visualize?
    # random x,y in [-5,5], z = half‐height = 0.25
    rx = np.random.uniform(-5, 5)
    ry = np.random.uniform(-5, 5)
    box.SetPos(chrono.ChVector3d(rx, ry, 0.25))
    # give it some friction
    box.GetMaterialSurfaceNSC().SetFriction(0.6)
    # give it a distinct color
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.2, 0.2))
    system.Add(box)

# ------------------------------------------------------------------
# 4) Create the TurtleBot
# ------------------------------------------------------------------
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# ------------------------------------------------------------------
# 5) Create a Sensor Manager and a spinning LiDAR on the chassis
# ------------------------------------------------------------------
manager = sens.ChSensorManager(system)
manager.SetVerbose(True)

# assume TurtleBot has a method GetBody() returning the main chassis ChBody
chassis = robot.GetBody()

# place the LiDAR 0.3 m above the chassis origin
lidar_offset = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0.3),
    chrono.ChQuaternionD(1, 0, 0, 0))

# a simple 360° planar LiDAR
lidar = sens.ChLidarSensor(
    chassis,
    updateRate=30,                # 30 Hz
    offsetPose=lidar_offset,
    horizontalSamples=360,
    verticalSamples=1,
    horizontalFov=chrono.CH_C_2PI,  # full 360°
    verticalFov=0.0,               # single layer
    maxDistance=10.0,
    minDistance=0.1)

# attach a simple buffer & a visualize filter
lidar.PushFilter(sens.ChFilterPCtoBuffer())
lidar.PushFilter(sens.ChFilterVisualize(1280, 720))

manager.AddSensor(lidar)

# ------------------------------------------------------------------
# 6) Create the Irrlicht visualizer
# ------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot w/ LiDAR and Random Boxes')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 1.5, 0.2),
    chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512)

# optional: vis.EnableShadows()

# ------------------------------------------------------------------
# 7) Motion‐control function for the TurtleBot
# ------------------------------------------------------------------
LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    """ mode = 'straight', 'left', 'right', or anything else to stop """
    if mode == 'straight':
        v = math.pi
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  v)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, v)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)
    else:
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)

# ------------------------------------------------------------------
# 8) Simulation loop
# ------------------------------------------------------------------
time_step = 2e-3
t = 0.0

while vis.Run():
    # drive straight all the time
    move('straight')

    # step the sensors
    manager.Update()

    # draw
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # advance dynamics
    system.DoStepDynamics(time_step)
    t += time_step