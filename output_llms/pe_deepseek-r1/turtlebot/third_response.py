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
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Corrected gravity direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create 5 randomly placed boxes
box_mat = chrono.ChContactMaterialNSC()
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetFixed(False)
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 100.0)

# Configure lidar sensor
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # body lidar is attached to
    10,                      # scanning rate in Hz
    lidar_offset,            # offset pose
    900,                     # number of horizontal samples
    30,                      # number of vertical channels
    math.radians(360),       # horizontal field of view
    math.radians(30),        # vertical field of view
    0.1,                     # minimum range
    100.0                    # maximum range
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.1)
lidar.SetCollectionWindow(0.01)

# Add lidar filters
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterAccess())
manager.AddSensor(lidar)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.VerticalDir_Z)  # Corrected enum
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot with Sensors and Obstacles')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Motion control function for Turtlebot
def move(mode):
    """Control Turtlebot movement direction"""
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    speed = math.pi  # rad/s
    
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, speed)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, speed)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, speed)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, speed)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Update movement based on time
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    else:
        move('right')
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation and update sensors
    system.DoStepDynamics(time_step)
    manager.Update()
    
    # Increment time counter
    time += time_step