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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
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

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.AddSensor(robot)

# Configure lidar sensor
lidar = sens.ChLidarSensor()
lidar.SetSensorMode(sens.ChLidarSensor.Mode_RANGE)
lidar.SetRangeMin(0.1)
lidar.SetRangeMax(10.0)
lidar.SetResolution(0.1)
lidar.SetFOV(math.pi / 2)
lidar.SetNoise(0.01)
lidar.SetSensorPosition(chrono.ChVector3d(0, 0, 0.1))
lidar.SetSensorRotation(chrono.ChQuaterniond(1, 0, 0, 0))
sensor_manager.AddSensor(lidar)

# Add randomly placed boxes
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box)

# Define motion control function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(0.5, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-0.5, 0)
        robot.SetMotorSpeed(0.5, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0.5, 0)
        robot.SetMotorSpeed(-0.5, 1)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    # At time = 1 s, start left turn
    if abs(time - 1.0) < 1e-4:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)

    # At time = 2 s, start right turn
    if abs(time - 2.0) < 1e-4:
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

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