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
sensor_manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 100), chrono.ChColor(1, 1, 1), 500)

# Create and add lidar sensor
lidar = sens.ChLidarSensor(robot.GetChassisBody(), 10, chrono.ChFrameD(chrono.ChVector3d(0.2, 0, 1), chrono.QuatFromAngleAxisD(0, chrono.ChVector3d(0, 1, 0))), 90, 300, 200)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.01)
lidar.PushFilter(sens.ChLidarPushFilter())
lidar.PushFilter(sens.ChLidarNoiseFilter(0.01, 0.001, 0.001))
lidar.PushFilter(sens.ChLidarIcpFilter())
sensor_manager.AddSensor(lidar)

# Create randomly placed boxes
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.2, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), -0.5))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)

# Motion control function for Turtlebot
def move(mode):
    left_speed = 0
    right_speed = 0
    if mode == 'straight':
        left_speed = math.pi
        right_speed = math.pi
    elif mode == 'left':
        left_speed = math.pi
        right_speed = 0
    elif mode == 'right':
        left_speed = 0
        right_speed = math.pi
    robot.SetMotorSpeed(left_speed, turtlebot.LEFT_DRIVE_WHEEL)
    robot.SetMotorSpeed(right_speed, turtlebot.RIGHT_DRIVE_WHEEL)

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
    # Update Turtlebot movement
    move('straight')
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update sensor manager
    sensor_manager.Update()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)
    time += time_step