import os
import math
import random # Added for random box placement
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens # Added for sensor manager and lidar

# Define WheelID constants globally (correction from original script)
# These are conventional IDs; turtlebot module might have turtlebot.LEFT_WHEEL_ID, etc.
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Define a maximum speed for the robot's wheels (rad/s)
ROBOT_MAX_SPEED = math.pi

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
# Set friction coefficient for the ground (example)
# ground_mat.SetFriction(0.8) 
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
# MODIFICATION 1: Ground position adjusted
ground.SetPos(chrono.ChVector3d(0, 0, -0.6)) # Position the ground
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# MODIFICATION 3: Added randomly placed boxes
num_boxes = 5
box_contact_mat = chrono.ChContactMaterialNSC() # Material for boxes
# box_contact_mat.SetFriction(0.5) # Example friction for boxes

# Determine ground surface Z for placing boxes
ground_thickness = 1.0 # As per ChBodyEasyBox third argument
ground_top_z = ground.GetPos().z + ground_thickness / 2.0

for i in range(num_boxes):
    size_x = random.uniform(0.2, 0.5)
    size_y = random.uniform(0.2, 0.5)
    size_z = random.uniform(0.2, 0.5)
    
    # Position boxes on top of the ground
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(-5, 5)
    pos_z = ground_top_z + size_z / 2.0
    
    box = chrono.ChBodyEasyBox(size_x, size_y, size_z, 1000, True, True, box_contact_mat)
    box.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    system.Add(box)

# MODIFICATION 4: Added motion control function for Turtlebot
def move(mode):
    """
    Controls the Turtlebot's movement.
    Modes: 'straight', 'left', 'right', 'stop'.
    """
    if mode == 'straight':
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED, ROBOT_MAX_SPEED)
    elif mode == 'left':
        # Gentle left turn: left wheel slower
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED * 0.5, ROBOT_MAX_SPEED)
        # For sharper turn: robot.SetMotorSpeedSync(-ROBOT_MAX_SPEED * 0.5, ROBOT_MAX_SPEED * 0.5)
    elif mode == 'right':
        # Gentle right turn: right wheel slower
        robot.SetMotorSpeedSync(ROBOT_MAX_SPEED, ROBOT_MAX_SPEED * 0.5)
        # For sharper turn: robot.SetMotorSpeedSync(ROBOT_MAX_SPEED * 0.5, -ROBOT_MAX_SPEED * 0.5)
    elif mode == 'stop':
        robot.SetMotorSpeedSync(0, 0)
    else:
        print(f"Unknown move mode: {mode}")
        robot.SetMotorSpeedSync(0, 0)


# MODIFICATION 2: Added sensor manager and configured lidar sensor
# Create a sensor manager
manager = sens.ChSensorManager(system)

# Define sensor update rate
sensor_update_rate = 10.0  # Hz

# Define the mounting pose of the lidar on the robot's chassis
# (relative to chassis frame: X forward, Y left, Z up)
# Let's place it 0.15m above the chassis center, facing forward
lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0, 0, 0.15), chrono.QUNIT)

# Create a Lidar sensor
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # Parent body: robot chassis
    sensor_update_rate,      # Update rate in Hz
    lidar_offset_pose,       # Offset pose
    360,                     # Number of horizontal samples
    1,                       # Number of vertical lines (for 2D planar lidar)
    2.0 * math.pi,           # Horizontal FOV
    0.02,                    # Vertical FOV (small for planar)
    10.0                     # Max range
)
lidar.SetName("LidarSensor")
lidar.SetLag(0.0) # No lag
lidar.SetMaxPollTime(0.001) # Max time for polling data

# Add filters for data access (e.g., point cloud)
# Filter for raw depth data (DI = Depth Information)
lidar.PushFilter(sens.ChFilterDIAccess())
# Filter to convert depth data to point cloud
lidar.PushFilter(sens.ChFilterPCfromDepth())
# Add lidar to the manager
manager.AddSensor(lidar)


# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot with Lidar and Obstacles')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2)) # Camera looking along Y
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (can be commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # MODIFICATION 5: Used move('straight') function for Turtlebot's movement
    move('straight') # Robot moves straight continuously

    # Old timed maneuvers removed as per instruction to use move('straight')
    # # At time = 1 s, start left turn
    # if abs(time - 1.0) < 1e-4:
    #     move('left')
        
    # # At time = 2 s, start right turn
    # if abs(time - 2.0) < 1e-4:
    #     move('right')

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # Increment time counter
    time += time_step

    # MODIFICATION 5: Added sensor manager update within the loop
    manager.Update()

    # Render the scene
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) # Clear with a color
    vis.Render()
    # Optionally, render sensor data (e.g., lidar points)
    # sens.ChSensorбычно_draw_point_cloud(vis, lidar, point_size=2.0) # If lidar has point cloud data directly accessible for vis
    vis.EndScene()


# Cleanup (optional, but good practice if resources need explicit release)
# No explicit cleanup needed for standard PyChrono objects typically
# vis.Quit() # Not available, Irrlicht window closes itself