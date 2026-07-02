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

# Create sensor manager and lidar sensor
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetAmbientLight(chrono.ChVector3d(0.5, 0.5, 0.5))
sensor_manager.SetShadowLight(chrono.ChVector3d(1.5, -2.5, 5.5), 3, 4, 10, 40, 512)

lidar = sens.ChLidar()
lidar.SetName("Lidar")
lidar.SetPosition(chrono.ChVector3d(0, 0.2, 0.5))  # Position relative to robot's chassis
lidar.SetDirection(chrono.ChVector3d(0, 0, 1))
lidar.SetRange(10)
lidar.SetAngularResolution(0.02)
lidar.SetHorizontalFOV(chrono.CH_C_PI)
lidar.SetVerticalFOV(chrono.CH_C_PI / 2)
lidar.SetUpdateRate(10)
lidar.SetFilter(sens.ChLidarFilterRange(0.1, 10))
sensor_manager.AddSensor(lidar)

# Create randomly placed boxes
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.5  # above ground
    box = chrono.ChBodyEasyBox(1, 1, 1, 100, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(False)
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()

# Define wheel IDs for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Define motion control function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-0.5, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-0.5, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.5, RIGHT_DRIVE_WHEEL)

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
    # Update sensor manager
    sensor_manager.Update()

    # Move the Turtlebot in straight line
    move('straight')

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)