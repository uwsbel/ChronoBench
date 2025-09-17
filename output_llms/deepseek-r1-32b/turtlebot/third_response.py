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
system.Add(sensor_manager)

# Add lidar sensor
lidar = sens.ChLidarSensor()
lidar.SetPosition(chrono.ChVector3d(0, 0.1, 0.5))  # Position relative to robot
lidar.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  # Rotation
lidar.SetNumberOfRays(360)
lidar.SetFieldOfView(chrono.CH_C_PI / 2)
lidar.SetRange(5.0)
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseFloor(0.01)
lidar.SetResolution(0.01)
lidar.SetVariance(0.001)
lidar.SetRenderingType(sens.ChLidarSensor.RENDERING_TYPE_POINTS)
lidar.SetUpdateRate(1.0 / 30.0)
lidar.SetParent(robot.GetBody())
sensor_manager.AddSensor(lidar)

# Create randomly placed boxes
for i in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetMaterialSurface(chrono.ChMaterialSurface())
    box.GetMaterialSurface().SetFriction(0.5)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/wood.jpg"))
    system.Add(box)

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

# Motion control function
def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-2.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-2.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2.0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Update sensor manager
    sensor_manager.Update()
    
    # Continuous straight movement
    move('straight')
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)
    
    # Increment time counter
    time += time_step