import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module import

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with adjusted position
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted Z position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Add randomly placed boxes
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(x, y, 0.0))
    box.SetFixed(False)
    system.Add(box)

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)

# Configure lidar sensor
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    robot.GetChassis(),
    update_rate=10,
    offset_pose=lidar_offset,
    horizontal_samples=1000,
    vertical_samples=32,
    horizontal_fov=2 * math.pi,
    vertical_fov=math.radians(30),
    max_vertical_angle=math.radians(15),
    min_vertical_angle=-math.radians(15),
    max_distance=100,
    lag=0,
    exposure_time=0,
    lens_model=sens.LensModelType_PINHOLE
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterLidarNoiseXYZ(0.01, 0.001))
lidar.PushFilter(sens.ChFilterLidarReduce(1))
sensor_manager.AddSensor(lidar)

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Modified Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

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

# Simulation parameters
time_step = 2e-3
time = 0
current_mode = 'straight'

# Simulation loop
while vis.Run():
    # Update movement mode based on time
    if time >= 1.0 and time < 2.0:
        current_mode = 'left'
    elif time >= 2.0:
        current_mode = 'right'
    
    move(current_mode)

    # Update sensor manager
    sensor_manager.Update()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)
    time += time_step