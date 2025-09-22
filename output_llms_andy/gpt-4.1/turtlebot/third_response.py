import os
import math
import numpy as np
import random
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

# -------------------------------
# 1. Create Chrono system
# -------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# -------------------------------
# 2. Create ground body as terrain plane
#    (Instruction 1: adjust Z from -1 to -0.6)
# -------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # <-- Adjusted Z position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# -------------------------------
# 3. Create Turtlebot Robot
# -------------------------------
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# -------------------------------
# 4. Add randomly placed boxes (Instruction 3)
# -------------------------------
box_mat = chrono.ChContactMaterialNSC()
box_size = [0.2, 0.2, 0.2]
for i in range(5):
    x = random.uniform(-4, 4)
    y = random.uniform(-4, 4)
    z = box_size[2] / 2 - 0.6  # Place on ground
    box = chrono.ChBodyEasyBox(box_size[0], box_size[1], box_size[2], 500, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetFixed(False)
    box.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    system.Add(box)

# -------------------------------
# 5. Sensor manager and lidar sensor (Instruction 2)
# -------------------------------
# Create sensor manager
manager = sens.ChSensorManager(system)

# Set up lidar sensor
lidar_update_rate = 10  # Hz
lidar_horiz_samples = 180
lidar_vert_samples = 1
lidar_horiz_fov = chrono.CH_C_PI  # 180 deg
lidar_vert_fov = 0.0
lidar_max_dist = 10.0
lidar_noise_std = 0.01

lidar_offset_pose = chrono.ChFrameD(chrono.ChVector3d(0.0, 0.0, 0.15))  # On top of robot

lidar = sens.ChLidarSensor(
    robot.GetBody(),                # Parent body
    lidar_update_rate,              # Update rate (Hz)
    lidar_offset_pose,              # Offset pose
    lidar_horiz_samples,            # Horizontal samples
    lidar_vert_samples,             # Vertical samples
    lidar_horiz_fov,                # Horizontal FOV (radians)
    lidar_vert_fov,                 # Vertical FOV (radians)
    lidar_max_dist                  # Max distance
)
lidar.SetName("Lidar Sensor")

# Add filters
lidar.PushFilter(sens.ChFilterLidarNoise(lidar_noise_std))
lidar.PushFilter(sens.ChFilterLidarAccess())  # Enables data access

manager.AddSensor(lidar)

# -------------------------------
# 6. Visualization
# -------------------------------
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
# vis.EnableShadows()  # Uncomment to enable shadows

# -------------------------------
# 7. Motion control function (Instruction 4)
# -------------------------------
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode):
    """Control Turtlebot movement mode: 'straight', 'left', 'right'."""
    speed = 1.0  # rad/s
    turn_speed = 1.0  # rad/s
    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(turn_speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(turn_speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)

# -------------------------------
# 8. Simulation loop (Instruction 5)
# -------------------------------
time_step = 2e-3
time = 0.0

while vis.Run():
    # Use motion control function
    move('straight')

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation
    system.DoStepDynamics(time_step)
    time += time_step