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

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.scene.Add(ground)

# Configure and add a lidar sensor
lidar = sens.ChLidarSensor(robot.GetChassisBody(),  # body lidar is attached to
                           10,  # scan rate in Hz
                           chrono.ChFrame(chrono.ChVector3d(0, 0, 0.2), chrono.Q_from_AngZ(0)),  # offset pose
                           100,  # number of horizontal samples
                           10,  # number of vertical channels
                           chrono.CH_C_PI,  # horizontal field of view
                           chrono.CH_C_PI / 8)  # vertical field of view
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIArea())
lidar.PushFilter(sens.ChFilterVisualize(1280, 720, 1))
sensor_manager.AddSensor(lidar)

# Add randomly placed boxes
box_mat = chrono.ChContactMaterialNSC()
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), 0.5))
    system.Add(box)
    sensor_manager.scene.Add(box)

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(0, 1)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    move('straight')  # Use the motion control function
    sensor_manager.Update()  # Update the sensor manager
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)
    time += time_step

# No modifications are needed here as the code is already provided above.