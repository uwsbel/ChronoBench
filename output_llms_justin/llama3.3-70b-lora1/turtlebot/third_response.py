import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
tb = robot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
tb.Initialize()  # Initialize the robot
system.Add(tb.GetChassisBody())

# Create sensor manager
manager = sens.ChSensorManager(system)
tb.PushFilterManager(manager)

noise_model_none = sens.ChNoiseModelNone()
noise_model_gpu = sens.ChNoiseModelNone()
cam_update_rate = 5
lidar_update_rate = 5
cam_fov = chrono.ChFramed(0.75, 0.75, 0.5, 0.5, -0.2, 1.57, -1.57)

intensity8 = sens.ChIntensityAccessArray()
manager.scene.AddPointLight(chrono.ChVector3f(0, 3, 0), chrono.ChColor(2, 2, 2), 5.0)
cam = sens.ChCameraSensor(
    tb.GetChassisBody(),
    update_rate,
    cam_fov,
    image_width,
    image_height,
    noise_model_none
    # cam_pushfilter
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)

manager.AddSensor(cam)

lidar = sens.ChLidarSensor(
    tb.GetChassisBody(),
    update_rate,
    horizontal,
    vertical,
    max_angle,
    min_angle,
    lidar_update_rate,
    offset,
    noise_model,
    update_rate,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow()

manager.AddSensor(lidar)

# Add boxes
box_mat = chrono.ChContactMaterialNSC()
box_length = 0.5
box_width = 0.5
box_height = 0.5
box_mass = 0
box_pos1 = chrono.ChVector3d(-2, -1, box_height / 2)
box_pos2 = chrono.ChVector3d(-1, 1, box_height / 2)
box_pos3 = chrono.ChVector3d(1, -1, box_height / 2)
box_pos4 = chrono.ChVector3d(1, 1, box_height / 2)
box_pos5 = chrono.ChVector3d(0, 0, box_height / 2)

box1 = chrono.ChBodyEasyBox(box_length, box_width, box_height, 1000, True, True, box_mat)
box1.SetPos(box_pos1)
box1.SetFixed(True)
box1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box1)

box2 = chrono.ChBodyEasyBox(box_length, box_width, box_height, 1000, True, True, box_mat)
box2.SetPos(box_pos2)
box2.SetFixed(True)
box2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box2)

box3 = chrono.ChBodyEasyBox(box_length, box_width, box_height, 1000, True, True, box_mat)
box3.SetPos(box_pos3)
box3.SetFixed(True)
box3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box3)

box4 = chrono.ChBodyEasyBox(box_length, box_width, box_height, 1000, True, True, box_mat)
box4.SetPos(box_pos4)
box4.SetFixed(True)
box4.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box4)

box5 = chrono.ChBodyEasyBox(box_length, box_width, box_height, 1000, True, True, box_mat)
box5.SetPos(box_pos5)
box5.SetFixed(True)
box5.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box5)

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

# Define wheel IDs
LEFT_WHEEL_ID = 0
RIGHT_WHEEL_ID = 1

def move(mode):
    if mode == "straight":
        tb.SetMotorSpeed(0.5, LEFT_WHEEL_ID)
        tb.SetMotorSpeed(0.5, RIGHT_WHEEL_ID)
    elif mode == "left":
        tb.SetMotorSpeed(0.5, LEFT_WHEEL_ID)
        tb.SetMotorSpeed(-0.5, RIGHT_WHEEL_ID)
    elif mode == "right":
        tb.SetMotorSpeed(-0.5, LEFT_WHEEL_ID)
        tb.SetMotorSpeed(0.5, RIGHT_WHEEL_ID)

# Simulation loop
time = 0
while vis.Run():
    move("straight")
    time += time_step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    manager.Update()
    system.DoStepDynamics(time_step)