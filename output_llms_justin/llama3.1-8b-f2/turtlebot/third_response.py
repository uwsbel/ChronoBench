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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
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

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    # At time = 1 s, start left turn
    if abs(time - 1.0) < 1e-4:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        
    # At time = 2 s, start right turn
    if abs(time - 2.0) < 1e-4:
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one timestep
    system.DoStepDynamics(time_step)

    # Update sensor manager
    sens_manager.Update()

    # Move the robot
    robot.move('straight')

# Create a sensor manager
# -----------------------
# Create a sensor manager attached to the ground body
# This manager will be used to add and update sensors
manager = sens.ChSensorManager(ground)
manager.scene.AddLight(chrono.ChVector3f(2, 2.5, 100).xyzla(), 1000000, 100)

# Add a lidar sensor
# ------------------
# Create a lidar sensor with default parameters
# The sensor will capture images of the scene in the driver's field of view
lidar = sens.ChLidarSensor(
    manager,
    100,  # update rate (FPS)
    300.,  # max horizontal/vertical range (meters)
    0.01,  # horizontal/vertical beam sample rate (meters)
    1.408,  # horizontal/vertical beam width (radians)
    100,    # number of horizontal/vertical beams
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    True,  # raw/filtered mode
    sens.LidarBeamShape_RECTANGULAR
)
lidar.PushFilter(sens.ChFilterLidarProcessRaw(sens.FilterOutlierByDistance(2, 10)))
manager.AddSensor(lidar)

# Add a camera sensor
# --------------------
# Create a camera sensor with default parameters
# The sensor will capture images of the scene in the driver's field of view
camera = sens.ChCameraSensor(
    manager,
    30,  # update rate (FPS)
    1.408,  # horizontal/vertical field of view (radians)
    1000.,  # width/height (pixels)
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    True,  # raw/filtered mode
)
camera.PushFilter(sens.ChFilterCameraProcessRaw(sens.FilterCompressJPEG(quality=5)))
manager.AddSensor(camera)

# Add a point cloud sensor
# ------------------------
# Create a point cloud sensor with default parameters
# The sensor will capture point clouds of the scene in the driver's field of view
pc_sensor = sens.ChPointCloudSensor(
    manager,
    10,  # update rate (FPS)
    300.,  # max horizontal/vertical range (meters)
    0.01,  # horizontal/vertical beam sample rate (meters)
    1.408,  # horizontal/vertical beam width (radians)
    100,    # number of horizontal/vertical beams
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
    0.1,  # max point cloud size
    True,  # raw/filtered mode
    sens.LidarBeamShape_RECTANGULAR
)
pc_sensor.PushFilter(sens.ChFilterPointCloudProcessRaw(sens.FilterResize(128, 128)))
manager.AddSensor(pc_sensor)

# Generate random boxes
# ---------------------
# Create 5 random boxes for the scene
# These boxes will be randomly positioned and will interact with the turtlebot
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), 1))
    box.SetFixed(False)
    box.SetMass(1000)
    system.Add(box)
    motor = chrono.ChLinkMotorRotationAngle()
    motor.Initialize(box, robot.GetChBody(), 0, 0)
    system.Add(motor)