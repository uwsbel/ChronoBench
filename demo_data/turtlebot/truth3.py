import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import numpy as np
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

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(5, 0.0, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance

robot.Initialize()  # Initialize the robot

# create number of boxes for robot to interact with
num_boxes = 5
for i in range(num_boxes):
    box = chrono.ChBodyEasyBox(1, 1, 1.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-10,10), np.random.uniform(-10,10), 0.5))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)
# Create sensor manager to monitor the scene
manager = sens.ChSensorManager(system)

# Create the lidar sensor and attach it to the ground
offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
# lidar related parameters
# Update rate in Hz
update_rate = 5.0
# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300
# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
# Lag time
lag = 0
# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate
# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2
# 3mm radius (as cited by velodyne)
divergence_angle = 0.003
# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
lidar = sens.ChLidarSensor(
    ground,              # Body lidar is attached to
    update_rate,            # Scanning rate in Hz
    offset_pose,            # Offset pose
    horizontal_samples,     # Number of horizontal samples
    vertical_samples,       # Number of vertical channels
    horizontal_fov,         # Horizontal field of view
    max_vert_angle,         # Maximum vertical field of view
    min_vert_angle,         # Minimum vertical field of view
    100.0,                  # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
    sample_radius,          # Sample radius
    divergence_angle,       # Divergence angle
    divergence_angle,       # Divergence angle (again, typically same value)
    return_mode             # Return mode for the lidar
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
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
time = 0
while vis.Run():
    move('straight')
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
