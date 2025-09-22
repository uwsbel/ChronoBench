import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens
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

# create boxes randomly placed
for i in range(5):
    box = chrono.ChBodyEasyBox(0.25, 0.25, 0.25, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(random.uniform(-3,3), random.uniform(-3,3), 0))
    box.SetFixed(True)  # Fix the ground in place
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(box)
# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot
manager = sens.ChSensorManager(system)

# Create a point light and add it to the sensor manager
light_point = sens.ChLightPoint()
light_point.SetPos(chrono.ChVector3d(2, 2.5, 10))
light_point.SetIntensity(1.0)
light_point.SetColor(chrono.ChColor(1, 1, 1))
manager.AddSensor(light_point)

# Create the LIDAR sensor and attach it to the robot's chassis
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

# Lidar sensor
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),              # Body lidar is assigned to
    0.01,                                       # Update rate in Hz
    offset_pose,                                # Offset pose
    180,                                       # Number of horizontal samples
    60,                                       # Number of vertical channels
    2 * chrono.CH_PI,                          # Horizontal field of view
    chrono.CH_PI / 12,                         # Maximum vertical field of view
    -chrono.CH_PI / 6,                         # Minimum vertical field of view
    10.0,                                      # Legacy parameter: max horizontal lidar range (at the horizon), in m
    sens.LidarBeamShape_RECTANGULAR,           # Beam shape
    1,                                         # Legacy parameter: number of samples per beam
    0.008,                                     # Legacy parameter: divergence half angle, in rad
    0.3,                                       # Legacy parameter: footprint diameter at 1 sample, in m
    false,                                     # Whether the lidar timing is conventional or not
    2,                                         # Number of raytracing samples to use for each beam
    20                                         # Sampling radius to use for WS scenes
)
lidar.SetName("Lidar Sensor")
# Set lag for this sensor
lidar.SetLag(0)
# Set collection window for the lidar
lidar.SetCollectionWindow(0.01)
# Apply a few filters
lidar.PushFilter(sens.ChFilterDIAngleTheta())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

manager.AddSensor(lidar)
# create three boxes randomly
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


def move(mode='straight'):
    """!
    Set motor speed to let turtlebot move in differently

    Args:
        mode (str, optional):  four modes: 'straight', 'left', 'right'. Defaults to 'straight'.
    """
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        return
# Simulation loop
time = 0
while vis.Run():
    move('straight')
    manager.Update()
    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)