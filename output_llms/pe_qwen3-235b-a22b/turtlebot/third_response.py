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
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with updated position
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Updated position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot with corrected initialization
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system)  # Corrected initialization
robot.Initialize(chrono.ChCoordsysd(init_pos, init_rot))  # Proper initialization with coordinate system
system.Add(robot)

# Create sensor manager and add lidar sensor
manager = sens.ChSensorManager(system)

# Create lidar sensor with multiple filters
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0.5, 0.2),  # Offset from robot chassis
    chrono.ChQuaterniond(1, 0, 0, 0)
)

lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # Attach to robot's chassis
    10,  # Update rate in Hz
    lidar_offset,
    360,  # Horizontal samples
    1,  # Vertical samples
    chrono.CH_C_PI,  # Horizontal field of view
    0,  # Vertical field of view
    10  # Maximum detection distance
)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud("Lidar Point Cloud"))
manager.AddSensor(lidar)

# Add random boxes to the environment
np.random.seed(42)  # For reproducibility
for i in range(5):
    box_size = 0.4
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True, ground_mat)
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = -0.6 + 0.5 + 0.5 * box_size  # Position on top of ground
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(False)
    box.EnableCollision(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 2e-3

# Define wheel IDs for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Motion control function
def move(mode):
    """Control Turtlebot movement based on mode"""
    if mode == 'straight':
        robot.SetMotorSpeed(2 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2 * math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(2 * math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(2 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Update sensor manager
    manager.Update()
    
    # Apply motion control
    if time < 1.0:
        move('straight')
    elif 1.0 <= time < 2.0:
        move('left')
    else:
        move('right')
    
    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation
    system.DoStepDynamics(time_step)