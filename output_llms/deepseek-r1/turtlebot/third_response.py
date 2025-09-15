import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create random boxes
for _ in range(5):
    x = np.random.uniform(-8, 8)
    y = np.random.uniform(-8, 8)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.scene.AddPointLight(chrono.ChVector3d(2, 2.5, 5), chrono.ChColor(2, 2, 2), 500)

# Configure lidar sensor
chassis = robot.GetChassisBody()
lidar_offset = chrono.ChVector3d(0, 0, 0.2)
lidar_transform = chrono.ChFramed(lidar_offset, chrono.Q_from_AngZ(0))
lidar = sens.ChLidarSensor(
    chassis,
    10,
    lidar_transform,
    1000,
    32,
    360,
    20,
    35,
    -15,
    100.0
)
lidar.SetName("Lidar")
lidar.PushFilter(sens.ChFilterLidarNoiseXYZ(0.01, 0.02))
lidar.PushFilter(sens.ChFilterLidarIntensityNoise(0.1))
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterLidarReduce(0.02))
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.5))
sensor_manager.AddSensor(lidar)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Modified Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()

# Wheel constants
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Motion control function
def move(robot, mode):
    if mode == 'straight':
        speed = math.pi
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time_step = 2e-3
while vis.Run():
    move(robot, 'straight')  # Set movement mode
    sensor_manager.Update()  # Update sensors
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)