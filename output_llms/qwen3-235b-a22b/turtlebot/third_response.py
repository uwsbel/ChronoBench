import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens  # Added for sensor support
from pychrono import irrlicht as chronoirr

# Define wheel IDs as constants
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position as per instruction
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Add randomly placed boxes
np.random.seed(42)  # Seed for reproducibility
num_boxes = 5
box_half_size = 0.25  # Assuming each box has size 0.5 in each dimension
box_material = chrono.ChContactMaterialNSC()
for i in range(num_boxes):
    # Generate random positions within the ground area, avoiding the robot's initial position
    x = np.random.uniform(-10, 10)
    y = np.random.uniform(-10, 10)
    # Place box on ground (ground top at z = -0.1, box half-size 0.25 in z)
    z_pos = -0.1 + box_half_size  # = -0.1 + 0.25 = 0.15
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_material)
    box.SetPos(chrono.ChVector3d(x, y, z_pos))
    box.SetFixed(False)
    system.Add(box)

# Create sensor manager and configure lidar
manager = sens.ChSensorManager(system)
manager.SetVerbose(0)  # Disable verbose output

# Create lidar sensor
lidar = sens.ChLidarSensor(
    robot,              # body to attach sensor to
    10,                 # scanning rate in Hz
    360,                # horizontal sample count
    1,                  # vertical sample count (2D lidar)
    chrono.CH_PI,       # horizontal field of view
    chrono.CH_PI / 12,  # vertical field of view
    100.0,              # max distance
    sens.LidarBeamShape_RECTANGULAR,  # beam shape
    2,                  # sample radius
    0.003,              # horizontal divergence
    0.003               # vertical divergence
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.1)  # Corresponds to 10Hz update rate
lidar.SetFilter(sens.ChLidarFilterNoiseNone())  # No noise
lidar.PushFilter(sens.ChLidarFilterVisualize())  # Visualize the data
manager.AddSensor(lidar)

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

# Set the simulation time step
time_step = 2e-3

# Define move function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-chrono.CH_PI, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-chrono.CH_PI, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Handle movement
    move('straight')  # Move straight continuously

    # Advance the simulation
    system.DoStepDynamics(time_step)

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Increment time
    time += time_step