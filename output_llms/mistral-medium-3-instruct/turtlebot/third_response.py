import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono import sensor as sens

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

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
system.Add(sensor_manager)

# Configure lidar sensor
lidar = sens.ChLidarSensor(robot.GetChassisBody(),  # parent body
                           10,                       # update rate in Hz
                           chrono.ChFrameD(chrono.ChVector3d(0.1, 0, 0.1), chrono.QUNIT),  # sensor offset
                           1.0,                      # max range
                           0.1,                      # min range
                           0.01,                     # angular resolution
                           360,                      # horizontal scan range
                           -30,                      # vertical scan range
                           30)                       # vertical scan range

# Add noise model and filters
noise_model = sens.ChGaussianNoiseModel(0.02, 0.001)
lidar.AddNoiseModel(noise_model)
lidar.AddFilter(sens.ChFilterAddOutlier(0.1))
lidar.AddFilter(sens.ChFilterAddGaussian(0.01))

sensor_manager.AddSensor(lidar)

# Create randomly placed boxes
np.random.seed(42)  # For reproducible random positions
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.3)
box_mat.SetRestitution(0.2)

for i in range(5):
    box = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 10, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-2, 2),
                                np.random.uniform(-2, 2),
                                np.random.uniform(0.1, 0.5)))
    box.SetRot(chrono.ChQuaterniond(np.random.uniform(0, 1),
                                   np.random.uniform(0, 1),
                                   np.random.uniform(0, 1),
                                   np.random.uniform(0, 1)))
    box.GetVisualShape(0).SetColor(chrono.ChColor(np.random.uniform(0, 1),
                                                 np.random.uniform(0, 1),
                                                 np.random.uniform(0, 1)))
    system.Add(box)

# Motion control function
def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

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

# Simulation loop
time = 0
while vis.Run():
    # Use the move function for straight movement
    move('straight')

    # Update sensor manager
    sensor_manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)