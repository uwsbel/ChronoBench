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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create randomly placed boxes for interaction
box_mat = chrono.ChContactMaterialNSC()
for i in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.25))
    box.SetFixed(True)
    system.Add(box)

# ---------------------------------------------------------------------
# Create the sensor manager and configure a lidar sensor
# ---------------------------------------------------------------------
manager = sens.ChSensorManager(system)

# Lidar parameters
update_rate = 5.0          # frequency of lidar updates [Hz]
horizontal_samples = 800   # number of horizontal samples
vertical_samples = 300     # number of vertical samples (channels)
horizontal_fov = 2 * math.pi
max_vert_angle = math.pi / 12
min_vert_angle = -math.pi / 6
lag = 0.0
collection_time = 1 / update_rate
sample_radius = 2
divergence_angle = 0.003
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Pose of the lidar relative to the robot chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# Create the lidar and attach it to the robot's chassis body
lidar = sens.ChLidarSensor(
    robot.GetChassis().GetBody(),  # body the lidar is attached to
    update_rate,
    offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    100.0,                         # maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    return_mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# Add filters to process the lidar data
lidar.PushFilter(sens.ChFilterDIAccess())        # access depth/intensity data
lidar.PushFilter(sens.ChFilterPCfromDepth())     # convert to point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())      # access XYZI data
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

# Add the lidar to the sensor manager
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

# Define WheelID enum values for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1


# ---------------------------------------------------------------------
# Motion control function for the Turtlebot
# ---------------------------------------------------------------------
def move(mode):
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
        raise ValueError("Invalid mode. Use 'straight', 'left', or 'right'.")


# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Drive the Turtlebot straight
    move('straight')

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the sensor manager
    manager.Update()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)