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
# Adjusted ground position to chrono.ChVector3d(0, 0, -0.6)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# -------------------------------  Sensor Manager  -------------------------------
manager = sens.ChSensorManager(system)

# -------------------------------  Lidar Sensor  -------------------------------
# Create a lidar sensor and add it to the sensor manager
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # Body to attach the sensor to
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.2)),  # Position and orientation on the body
    math.pi / 4.0,  # Horizontal FOV
    # Vertical FOV
    chrono.CH_C_PI / 12,  # Update rate in Hz
    50,  # Number of horizontal rays
    5,  # Number of vertical rays
    20.0,  # Maximum distance in meters
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)  # Set the lag (delay) in seconds
lidar.AddNoise(0.01)  # Add Gaussian noise with standard deviation of 0.1 meters
lidar.SetOffset(0.01, 0.1)  # Add a constant offset to the measurements
# lidar.Filter()
# lidar.Blur()
# lidar.Downsample()
manager.AddSensor(lidar)

# -------------------------------  Random Boxes  -------------------------------
# Function to generate a random position within a specified range
def generate_random_pos(x_range, y_range, z_range):
    x = np.random.uniform(x_range[0], x_range[1])
    y = np.random.uniform(y_range[0], y_range[1])
    z = np.random.uniform(z_range[0], z_range[1])
    return chrono.ChVector3d(x, y, z)

# Add boxes at random positions
num_boxes = 5
for i in range(num_boxes):
    size = np.random.uniform(0.2, 0.5)
    box = chrono.ChBodyEasyBox(size, size, size, 1000, True, True, ground_mat)
    box.SetPos(generate_random_pos((-5, 5), (-5, 5), (0.5, 1.5)))
    system.Add(box)


# -------------------------------  Robot Motion Control  -------------------------------
def move(mode):
    """
    Controls the Turtlebot's movement.

    Args:
        mode (str): 'straight', 'left', 'right'
    """
    if mode == "straight":
        robot.SetMotorSpeed(math.pi / 2, 0)  # Left wheel speed
        robot.SetMotorSpeed(math.pi / 2, 1)  # Right wheel speed
    elif mode == "left":
        robot.SetMotorSpeed(math.pi / 4, 0)
        robot.SetMotorSpeed(-math.pi / 4, 1)
    elif mode == "right":
        robot.SetMotorSpeed(-math.pi / 4, 0)
        robot.SetMotorSpeed(math.pi / 4, 1)


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

# Simulation loop
time = 0
while vis.Run():
    move("straight")  # Move the robot straight

    # Update the sensor manager
    manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)