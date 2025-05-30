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

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

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

# Create sensor manager
manager = sens.ChSensorManager(system)

# Configure and add a lidar sensor
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # body lidar is attached to
    10,  # scanning rate in Hz
    chrono.ChFrame(chrono.ChVector3d(0, 0, .5), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),  # offset pose
    100,  # number of horizontal samples (the more beams the higher the resolution)
    30,  # number of vertical channels (the more channels the larger the vertical field of view)
    chrono.CH_C_PI,  # horizontal field of view (radians)
    chrono.CH_C_PI / 6.,  # vertical field of view (radians)
    0.1,  # near plane (m)
    100  # far plane (m)
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(256, 256, "Lidar Depth Data"))
manager.AddSensor(lidar)

# Add randomly placed boxes
for _ in range(5):
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box)

# Define motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi / 2, 0)  # Corrected argument order
        robot.SetMotorSpeed(math.pi / 2, 1)  # Corrected argument order
    elif mode == 'left':
        robot.SetMotorSpeed(0, 0)  # Corrected argument order
        robot.SetMotorSpeed(math.pi, 1)  # Corrected argument order
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)  # Corrected argument order
        robot.SetMotorSpeed(0, 1)  # Corrected argument order

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Use the move function to control Turtlebot's movement
    if time < 1.0:
        move('straight')
    elif abs(time - 1.0) < 1e-4:
        move('left')
    elif abs(time - 2.0) < 1e-4:
        move('right')

    # Increment time counter
    time += time_step

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)