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
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetVisualize(True)

# Add lidar sensor
lidar = sens.ChLidar()
lidar.SetPosition(chrono.ChVector3d(0, 0, 0.5), robot.GetBody())  # Position relative to robot
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
lidar.SetNumberOfRays(360)
lidar.SetFieldOfView(chrono.CH_C_PI / 2)
lidar.SetRange(5.0)
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseFloor(0.01)
lidar.SetVariance(0.001)
lidar.SetPointsCloudColor(chrono.ChColor(0.5, 0.5, 0.5))
lidar.SetRenderingType(sens.ChLidar.RENDERING_TYPE_POINTS)
sensor_manager.AddSensor(lidar)

# Create run-time visualization
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

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Add random boxes
for i in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetMaterialSurface(ground_mat)
    system.Add(box)

# Define motion control function
def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update sensor manager
    sensor_manager.Update()

    # Control robot movement
    move('straight')

    # Increment time counter
    time += time_step

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)