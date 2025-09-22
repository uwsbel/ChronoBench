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

# Define motion modes for the robot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.2, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.2, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0.2, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-0.2, RIGHT_DRIVE_WHEEL)
    else:
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Create a sensor manager
sens_manager = sens.ChSensorManager(system)
sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(2, 2, 2), 500)
sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(2, 2, 2), 500)
sens_manager.scene.AddAreaLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(2, 2, 2), 500, chrono.ChVector3f(0,0,1))

# Add a lidar sensor
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    robot.GetChBody(),              # body lidar is attached to
    update_rate,                    # scanning rate in Hz
    offset_pose,                    # offset pose
    horizontal_samples,             # number of horizontal samples
    vertical_samples,               # number of vertical channels
    horizontal_fov,                 # horizontal field of view
    max_vert_angle,                 # vertical field of view
    min_vert_angle,
    100.0,                          # max lidar range
    sens.LidarBeamShape_RECTANGULAR,  # beam shape (RECTANGULAR or CYLINDRICAL)
    sample_radius,                  # sample radius (only for RECTANGULAR shape)
    divergence_angle,               # divergence angle
    divergence_angle,               # divergence angle
    return_mode                     # return mode for the lidar
)
lidar.PushFilter(sens.ChFilterLidarProcess())
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)
sens_manager.AddSensor(lidar)

# Add a camera sensor
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(
    robot.GetChBody(),              # body camera is attached to
    update_rate,                    # update rate in Hz
    offset_pose,                    # offset pose
    image_width,                    # image width
    image_height,                   # image height
    fov                             # camera's horizontal field of view
)
camera.PushFilter(sens.ChFilterCameraProcess())
camera.SetName("Camera Sensor")
camera.SetLag(lag)
sens_manager.AddSensor(camera)

# Add a GPS sensor
gps = sens.ChGPSSensor(robot.GetChBody(), update_rate, 100.0)
gps.PushFilter(sens.ChFilterGPSProcess())
gps.SetName("GPS Sensor")
gps.SetLag(lag)
sens_manager.AddSensor(gps)

# Add an IMU sensor
imu = sens.ChIMUSensor(robot.GetChBody(), update_rate, 100.0, 9e9)
imu.PushFilter(sens.ChFilterIMUProcess())
imu.SetName("IMU Sensor")
imu.SetLag(lag)
sens_manager.AddSensor(imu)

# Add a scanner sensor (point cloud)
offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
scanner = sens.ChScannerSensor(
    robot.GetChBody(),              # body scanner is attached to
    update_rate,                    # scanning rate in Hz
    offset_pose,                    # offset pose
    horizontal_samples,             # number of horizontal samples
    vertical_samples,               # number of vertical channels
    horizontal_fov,                 # horizontal field of view
    max_vert_angle,                 # vertical field of view
    min_vert_angle,
    100.0,                          # max sensor range
    sens.ScannerBeamShape_RECTANGULAR,  # beam shape (RECTANGULAR or CYLINDRICAL)
    sample_radius,                  # sample radius (only for RECTANGULAR shape)
    divergence_angle,               # divergence angle
    divergence_angle,               # divergence angle
    return_mode                     # return mode for the scanner
)
scanner.PushFilter(sens.ChFilterScannerProcess())
scanner.SetName("Scanner Sensor")
scanner.SetLag(lag)
scanner.SetCollectionWindow(collection_time)
sens_manager.AddSensor(scanner)

# Render the sensor data
def render():
    sens_manager.BeginScene()
    sens_manager.Render()
    sens_manager.EndScene()

# Update function for the sensor manager
def Update():
    time = robot.GetSystem().GetChTime()
    if abs(time - 1.0) < 1e-4:
        lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(1.5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    if abs(time - 2.0) < 1e-4:
        lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(-1.5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sens_manager.Update()

# Add some boxes to interact with the sensors
boxes = []
for i in range(5):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(chrono.ChVector3d(0.5 + (i % 2) * 1.0, 0, 1.5 + (i // 2) * 1.0))
    box.SetFixed(False)
    box.SetMass(1000)
    system.Add(box)
    boxes.append(box)

# Simulation loop
render()
while vis.Run():
    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation for all systems (drive the simulation for all systems forward in time for this timestep)
    system.DoStepDynamics(time_step)

    # Update sensor manager
    sens_manager.Update()

    # Move robot
    move('straight')

    # Check for collisions
    for box in boxes:
        if box.GetPos().y - 0.3 < robot.GetChBody().GetPos().y and box.GetPos().y + 0.3 > robot.GetChBody().GetPos().y and abs(box.GetPos().x - robot.GetChBody().GetPos().x) < 0.5:
            robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
        if box.GetPos().x + 0.3 > robot.GetChBody().GetPos().x and box.GetPos().x - 0.3 < robot.GetChBody().GetPos().x and box.GetPos().z - 0.3 < robot.GetChBody().GetPos().z and box.GetPos().z + 0.3 > robot.GetChBody().GetPos().z:
            robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
            robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)