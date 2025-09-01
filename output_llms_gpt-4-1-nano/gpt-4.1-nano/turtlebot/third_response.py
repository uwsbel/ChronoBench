import os
import math
import random
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Import sensor module

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetG_acc(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction

# Set default collision envelope and margin
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
# 1. Adjust ground position as per instruction
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Set up Irrlicht visualization
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
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Optional: enable shadows (commented out)
# vis.EnableShadows()

# 2. Setup Sensor Manager and Lidar sensor
sensor_manager = sens.ChSensorManager(system)
# Create a laser scanner sensor attached to the robot
lidar_params = sens.ChLidarSensor(
    robot.GetChassisBody(),
    0.2,               # position of sensor relative to robot chassis
    0,                 # yaw angle
    270,               # horizontal field of view degrees
    0.1,               # vertical field of view degrees
    720,               # number of horizontal samples
    0.2,               # max range
    sens.ChLidarSensor.SENSITIVITY_HIGH,
    0.1,               # update rate in seconds
    sens.ChSensor.SENSOR_TYPE_LIDAR
)
# Optional: set additional parameters
lidar_params.SetDirection(chrono.ChVector3d(1, 0, 0))
sensor_manager.Add(lidar_params)

# 3. Add randomly placed boxes
num_boxes = 5
boxes = []
for _ in range(num_boxes):
    size = random.uniform(0.2, 0.5)
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(-5, 5)
    box_mat = chrono.ChMaterialSurfaceNSC()
    box = chrono.ChBodyEasyBox(size, size, size, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(pos_x, pos_y, size/2))
    system.Add(box)
    boxes.append(box)

# 4. Define motion control function
def move(mode):
    # modes: 'straight', 'left', 'right'
    if mode == 'straight':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(math.pi/4, 0)
        robot.SetMotorSpeed(-math.pi/4, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi/4, 0)
        robot.SetMotorSpeed(math.pi/4, 1)

# 5. Simulation loop parameters
time_step = 2e-3
time = 0

while vis.Run():
    # Use move() function for robot control
    # Example: move straight for first second, then turn, etc.
    if time < 1:
        move('straight')
    elif 1 <= time < 2:
        move('left')
    elif 2 <= time < 3:
        move('right')
    else:
        move('straight')

    # Update sensor manager
    sensor_manager.Update()

    # Increment time
    time += time_step

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)