import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module import

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

# Add sensor manager and lidar sensor
sensor_manager = sens.ChSensorManager(system)
robot.GetBody().AddSensor(sensor_manager)

# Configure lidar sensor
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVector3d(0, 0.15, 0.3))  # Position relative to robot
lidar.SetRange(10.0)  # Maximum distance
lidar.SetHorizontalFOV(math.pi * 0.5)  # 90 degree field of view
lidar.SetVerticalFOV(math.pi * 0.25)  # 45 degree vertical field of view
lidar.SetResolution(0.1)  # Angular resolution
lidar.SetCoordinateSystem(chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

# Add filters
lidar.AddFilter(sens.ChLidarFilterMinRange(0.1))
lidar.AddFilter(sens.ChLidarFilterMaxRange(10.0))
lidar.AddFilter(sens.ChLidarFilterPointsBehindSensor())

sensor_manager.AddSensor(lidar)

# Create randomly placed boxes
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
    box.SetPos(chrono.ChVector3d(
        np.random.uniform(-8, 8),
        np.random.uniform(-8, 8),
        0.25
    ))
    box.SetMaterialSurface(ground_mat)
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
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Update sensor manager
    sensor_manager.Update()
    
    # Get lidar data
    lidar_data = lidar.GetPoints()
    # print("Lidar points:", lidar_data)  # Uncomment to see sensor data
    
    # Simple movement pattern
    if time < 10:
        move('straight')
    elif time < 15:
        move('left')
    else:
        move('right')
    
    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)