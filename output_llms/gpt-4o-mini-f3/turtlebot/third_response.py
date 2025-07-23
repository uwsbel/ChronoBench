import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import random

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

# create randomly placed boxes for robot to interact with
num_boxes = 5
for i in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 1.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(random.uniform(-5, 5), random.uniform(-5, 5), 0.25))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# create sensor manager and add sensors
manager = sens.ChSensorManager(system)

# Create the lidar sensor
# Create the lidar sensor
offset_pose = chrono.ChFramed(
        chrono.ChVector3d(2.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
 # Update the lidar's pose relative to the body it is attached to
lidar = sens.ChLidarSensor(
    robot.GetRobot(),              # Sensor attached to this body
    update_rate,                  # Sensor update rate in Hz
    offset_pose,                  # Offset pose
    horizontal_samples,           # Number of horizontal samples
    vertical_samples,             # Number of vertical channels
    horizontal_fov,               # Horizontal field of view
    max_vert_angle,               # Maximum vertical field of view
    min_vert_angle,               # Minimum vertical field of view
    100.0,                        # Maximum lidar range
    sens.LidarMode_STANDARD       # Lidar mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# Provide the filter graph for the sensor
lidar.PushFilter(sens.ChFilterDIAttitudeBias())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))



# Add the sensor to the manager
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

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 2e-3

# simulation time
time = 0

# function to control the motion of turtlebot
def move(mode):
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi/2, LEFT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi/2, RIGHT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
    else:
        raise ValueError('Invalid mode')
# Simulation loop
while vis.Run():
    
    
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    else:
        move('right')
    
    
    # Increment time counter
    time += time_step
    
    
    # update sensor manager
    manager.Update()
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)