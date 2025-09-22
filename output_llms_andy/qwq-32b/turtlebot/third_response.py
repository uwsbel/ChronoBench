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

# Create ground body as terrain plane
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Modified')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Create sensor manager and configure lidar
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetVisualSystem(vis)
sensor_manager.SetRenderingCamera(vis.GetCamera())

# Add lidar sensor
robot_body = robot.GetBody()
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPos(chrono.ChVectorD(0, 0, 0.1))
lidar.SetRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0)))  # Adjust orientation
lidar.SetUpdateRate(10)
lidar.SetRange(5.0)
lidar.SetHorizontalResolution(0.5)
lidar.SetVerticalResolution(1.0)
lidar.SetFOV(270, 30)
lidar.SetRangeResolution(0.01)
lidar.SetClippingPlanes(0.1, 100)
lidar.PushFilter(sens.ChLidarFilterRange(0.1, 5.0))
lidar.PushFilter(sens.ChLidarFilterVoxelGrid(0.1))
lidar.PushFilter(sens.ChLidarFilterPassThrough(chrono.ChVectorD(-5, -5, -2), chrono.ChVectorD(5, 5, 2)))
lidar.SetParentBody(robot_body)
sensor_manager.AddSensor(lidar)

# Add randomly placed boxes
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.5
    box = chrono.ChBodyEasyBox(1, 1, 0.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVectorD(x, y, z))
    system.Add(box)

def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

time_step = 2e-3
time = 0

# Simulation loop
while vis.Run():
    # Determine movement mode based on time
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    else:
        move('right')

    # Update simulation
    system.DoStepDynamics(time_step)
    sensor_manager.Update()  # Update sensors

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    time += time_step