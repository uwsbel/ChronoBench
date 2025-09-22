import os
import math
import random
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector(0, 0, -0.6))  # Adjusted ground position as instructed
ground.SetBodyFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaternion(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create randomly placed boxes
box_mat = chrono.ChMaterialSurfaceNSC()
for i in range(5):
    # Random position for boxes, keeping them on the ground and in view
    rand_x = random.uniform(-5, 5)
    rand_y = random.uniform(-5, 5)
    
    # Create box
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector(rand_x, rand_y, 0))
    box.SetBodyFixed(False)  # Allow boxes to move if hit
    
    # Add box to system
    system.Add(box)

# Set up sensor manager
sens_manager = sens.ChSensorManager(system)
sens_manager.scene.AddPointLight(chrono.ChVector(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)

# Set up lidar sensor
lidar_noise = sens.ChNoiseNone() # No noise for lidar
lidar_offset_pose = chrono.ChFrame(chrono.ChVector(0, 0, 0.3), chrono.ChQuaternion(1, 0, 0, 0))
intensity_mode = True
horizontal_samples = 1000
vertical_samples = 16
horizontal_fov = 2 * math.pi
vertical_fov = math.pi / 6
max_vert_angle = vertical_fov / 2
min_vert_angle = -vertical_fov / 2
max_distance = 100
min_distance = 0.1
lidar_update_rate = 10

lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),                 # body lidar is attached to
    lidar_update_rate,                       # update rate in Hz
    lidar_offset_pose,                       # offset pose
    horizontal_samples,                      # number of horizontal samples
    vertical_samples,                        # number of vertical channels
    horizontal_fov,                          # horizontal field of view
    min_vert_angle,                          # minimum vertical angle
    max_vert_angle,                          # maximum vertical angle
    max_distance,                            # max distance
    intensity_mode                           # intensity mode
)

# Add noise model to the lidar
lidar.PushFilter(lidar_noise)
# Add data processing for visualization
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))

# Add the lidar to the sensor manager
sens_manager.AddSensor(lidar)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(0, 1.5, 0.2), chrono.ChVector(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(1.5, -2.5, 5.5), 3, chrono.ChColor(0.7, 0.7, 0.7))

# Set the simulation time step
time_step = 2e-3

# Define WheelID enum values for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # At time = 1 s, start left turn
    if abs(time - 1.0) < 1e-4:
        move('left')
        
    # At time = 2 s, start right turn
    elif abs(time - 2.0) < 1e-4:
        move('right')
    
    # At time = 3 s, go straight
    elif abs(time - 3.0) < 1e-4:
        move('straight')
    
    # Initialize with straight movement
    elif time < 1.0:
        move('straight')

    # Increment time counter
    time += time_step

    # Update sensor manager
    sens_manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)