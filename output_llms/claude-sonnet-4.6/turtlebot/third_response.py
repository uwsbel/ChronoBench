import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

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

# Add randomly placed boxes for interaction
np.random.seed(42)
box_mat = chrono.ChContactMaterialNSC()
for i in range(5):
    box_x = float(np.random.uniform(-4, 4))
    box_y = float(np.random.uniform(-4, 4))
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(box_x, box_y, 0.0))
    box.SetFixed(True)
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Define WheelID enum values for motor control (outside loop for efficiency)
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Define movement control function
def move(mode):
    """Control Turtlebot movement with modes: 'straight', 'left', 'right'."""
    if mode == 'straight':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# -------------------------------------------------------
# Create and configure sensor manager
# -------------------------------------------------------
sensor_manager = sens.ChSensorManager(system)
sensor_manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    5000
)

# Lidar sensor parameters
lidar_update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * math.pi          # Full 360-degree horizontal FOV
max_vert_angle = math.pi / 12         # Max vertical angle (~15 deg)
min_vert_angle = -math.pi / 6         # Min vertical angle (~-30 deg)
max_range = 100.0                      # Max lidar range (m)
lag = 0.0
collection_time = 1.0 / lidar_update_rate

# Lidar offset pose relative to robot chassis
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# Create lidar sensor attached to the robot chassis
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),       # Parent body
    lidar_update_rate,            # Update rate (Hz)
    lidar_offset_pose,            # Offset pose
    horizontal_samples,           # Horizontal samples
    vertical_samples,             # Vertical samples
    horizontal_fov,               # Horizontal field of view
    max_vert_angle,               # Max vertical angle
    min_vert_angle,               # Min vertical angle
    max_range,                    # Max range
    sens.LidarBeamShape_RECTANGULAR,  # Beam shape
    2,                            # Sample radius
    0.003,                        # Divergence angle
    0.003,                        # Divergence angle (vertical)
    sens.LidarReturnMode_STRONGEST_RETURN  # Return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# Add filters to lidar sensor
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterXYZIToBuf())
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Point Cloud"))

# Add lidar to sensor manager
sensor_manager.AddSensor(lidar)

# -------------------------------------------------------
# Create run-time visualization
# -------------------------------------------------------
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
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)

# Set the simulation time step
time_step = 2e-3

# -------------------------------------------------------
# Simulation loop
# -------------------------------------------------------
time = 0
while vis.Run():
    # Control Turtlebot to move straight
    move('straight')

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update sensor manager
    sensor_manager.Update()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # Increment time counter
    time += time_step