import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
if system is None:
    raise Exception("Failed to create Chrono system")

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
if ground is None:
    raise Exception("Failed to create ground body")

ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
if robot is None:
    raise Exception("Failed to create Turtlebot instance")

robot.Initialize()  # Initialize the robot

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
if sensor_manager is None:
    raise Exception("Failed to create sensor manager")

# Configure and add lidar sensor
lidar_sensor = sens.ChLidarSensor(chrono.ChFrame3d(chrono.ChVector3d(0.2, 0, 0.5)))
lidar_sensor.Setup(num_horizontal_channels=360, num_vertical_channels=10, horizontal_min_angle=-math.pi, horizontal_max_angle=math.pi, vertical_min_angle=-math.pi/4, vertical_max_angle=math.pi/4, min_distance=0.1, max_distance=10.0)
lidar_sensor.SetUpdateRate(10)
sensor_manager.AddSensor(lidar_sensor)

# Create randomly placed boxes
for _ in range(5):
    box_body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box_body)

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, 0)  # Left wheel
        robot.SetMotorSpeed(0.5, 1)  # Right wheel
    elif mode == 'left':
        robot.SetMotorSpeed(-0.5, 0)  # Left wheel
        robot.SetMotorSpeed(0.5, 1)  # Right wheel
    elif mode == 'right':
        robot.SetMotorSpeed(0.5, 0)  # Left wheel
        robot.SetMotorSpeed(-0.5, 1)  # Right wheel

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

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    try:
        move('straight')  # Control Turtlebot's movement

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
    except Exception as e:
        print(f"Error during simulation: {e}")
        break