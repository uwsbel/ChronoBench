import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

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
robot = turtlebot.TurtleBot(system, init_pos, init_rot, True, True)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

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

# Add randomly placed boxes
num_boxes = 5
for i in range(num_boxes):
    # Random position within a range
    pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 1)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box.SetPos(pos)
    box.SetMass(10)
    system.Add(box)

# Create and configure sensor manager and lidar sensor
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetName("lidar_manager")

# Configure lidar sensor
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetRange(50.0)
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetFovHorizontal(180)
lidar.SetFovVertical(30)
lidar.SetPosition(chrono.ChVector3d(0, 0.5, 1.0))  # Position relative to robot
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  # Orientation

# Add filters
range_filter = sens.ChLidarRangeFilter()
range_filter.SetMinRange(0.1)
range_filter.SetMaxRange(50.0)
lidar.AddFilter(range_filter)

intensity_filter = sens.ChLidarIntensityFilter()
intensity_filter.SetMinIntensity(0.1)
intensity_filter.SetMaxIntensity(1.0)
lidar.AddFilter(intensity_filter)

sensor_manager.AddSensor(lidar)

# Define motion control function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2.0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Start with straight movement
    move('straight')

    # Update sensor manager
    sensor_manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)