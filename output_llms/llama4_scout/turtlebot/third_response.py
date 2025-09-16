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

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)

# Configure and add lidar sensor
lidar_pos = chrono.ChVector3d(0, 0, 0.5)
lidar_rot = chrono.ChQuaterniond(1, 0, 0, 0)
lidar_sensor = sens.ChLidarSensor(robot.GetBody(), lidar_pos, lidar_rot, 0.5, 0.1, 100)
lidar_sensor.Update()  # Initialize the lidar points vector
sensor_manager.AddSensor(lidar_sensor)

# Create and add 5 boxes with random positions
for _ in range(5):
    box_body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box_body.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box_body)

# Function to control Turtlebot movement
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, 0)  # Both wheels forward
        robot.SetMotorSpeed(0, 1)  # Both wheels forward
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, 0)  # Left wheel backward, right wheel forward
        robot.SetMotorSpeed(math.pi, 1)  # Left wheel backward, right wheel forward
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)  # Left wheel forward, right wheel backward
        robot.SetMotorSpeed(-math.pi, 1)  # Left wheel forward, right wheel backward

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

# Simulation parameters
time_step = 2e-3
time = 0

# Simulation loop
while vis.Run():
    # Control Turtlebot movement
    move('straight')  # Move straight

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