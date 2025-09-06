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
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager and configure lidar sensor
sensor_manager = sens.ChSensorManager(system)
sensor_manager.AddSensor(robot.GetLidarSensor())

# Configure lidar sensor
lidar_sensor = robot.GetLidarSensor()
lidar_sensor.SetName("lidar_sensor")
lidar_sensor.SetPos(chrono.ChVector3d(0, 0, 0.5))
lidar_sensor.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
lidar_sensor.SetRange(10.0)
lidar_sensor.SetResolution(0.5)
lidar_sensor.SetScanRate(10.0)
lidar_sensor.SetNoiseStdDev(0.01)
lidar_sensor.SetMinRange(0.1)
lidar_sensor.SetMaxRange(10.0)
lidar_sensor.SetFieldOfView(chrono.CH_C_PI / 2)
lidar_sensor.SetRayFilteringMode(chrono.ChLidarSensor.RAYFILTERING_ALL)

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

# Create randomly placed boxes
np.random.seed(0)  # Seed for reproducibility
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.25)
    box.SetPos(box_pos)
    box.SetFixed(True)
    system.Add(box)

# Function to control Turtlebot's movement
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("Unknown movement mode")

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    # Use move function for Turtlebot's movement
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