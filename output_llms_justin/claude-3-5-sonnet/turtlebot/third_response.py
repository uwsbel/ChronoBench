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
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  # Adjusted ground position
ground.SetBodyFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVectorD(0, 0, 0.2)  # Initial position of the robot
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(1, 1, 1), 500)

# Create lidar sensor
offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # body lidar is attached to
    20,                      # scanning rate in Hz
    offset_pose,             # offset pose relative to body
    1000,                    # number of horizontal samples
    50,                      # number of vertical channels
    chrono.CH_C_PI,          # horizontal field of view
    chrono.CH_C_PI / 6.,     # vertical field of view
    100.0                    # max distance
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add noise model
lidar.PushFilter(sens.ChFilterLidarNoiseSimple(0.01, 0.001))
# Add a visualization filter
lidar.PushFilter(sens.ChFilterVisualize(1000, "Lidar Data"))
# Add lidar to the sensor manager
manager.AddSensor(lidar)

# Create randomly placed boxes
box_material = chrono.ChMaterialSurfaceNSC()
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True, box_material)
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box.SetPos(chrono.ChVectorD(x, y, 0))
    system.Add(box)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 2e-3

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Move the Turtlebot straight
    move('straight')

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