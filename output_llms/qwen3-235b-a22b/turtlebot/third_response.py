import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module

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

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Define wheel IDs for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Function to control Turtlebot movement
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Create sensor manager and add lidar
manager = sens.ChSensorManager(robot)
manager.SetAsynchronous(False)

# Configure and add lidar sensor
lidar = sens.ChLidarSensor(
    robot,
    10, 360, 1,
    chrono.CH_C_PI, chrono.CH_C_PI / 12, 10
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.1)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud("Point Cloud"))
manager.AddSensor(lidar)

# Add randomly placed boxes
for i in range(5):
    box_size = 0.5
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, ground_mat)
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box.SetPos(chrono.ChVector3d(x, y, box_size / 2 + 0.01))
    box.SetFixed(False)
    system.Add(box)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Control robot movement
    move('straight')  # Move straight throughout simulation

    # Increment time
    time += time_step

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update sensor manager
    manager.Update()