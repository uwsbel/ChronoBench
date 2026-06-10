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
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# ---------------------------------------------------------------------
# Create the sensor manager
# ---------------------------------------------------------------------
manager = sens.ChSensorManager(system)

# Lidar parameters
update_rate = 5.0           # Hz
horizontal_samples = 800    # number of horizontal samples
vertical_samples = 300      # number of vertical channels
horizontal_fov = 2 * chrono.CH_PI          # 360 degree horizontal FOV
max_vert_angle = chrono.CH_PI / 12         # +15 degrees
min_vert_angle = -chrono.CH_PI / 6         # -30 degrees
lag = 0.0
collection_time = 1.0 / update_rate        # collect for one update cycle
sample_radius = 2                          # number of rays used for beam divergence
divergence_angle = 0.003                   # beam divergence
max_distance = 100.0                       # max lidar range

# Offset pose of the lidar relative to the chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# Create the lidar sensor attached to the rover chassis
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),   # body to which the sensor is attached
    update_rate,                    # update rate in Hz
    offset_pose,                    # offset pose
    horizontal_samples,             # number of horizontal samples
    vertical_samples,               # number of vertical channels
    horizontal_fov,                 # horizontal field of view
    max_vert_angle,                 # max vertical angle
    min_vert_angle,                 # min vertical angle
    max_distance,                   # max lidar range
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    sens.LidarReturnMode_STRONGEST_RETURN
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# Add filters to the lidar data
lidar.PushFilter(sens.ChFilterDIAccess())          # access raw depth/intensity data
lidar.PushFilter(sens.ChFilterPCfromDepth())       # convert depth to point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())        # access point cloud data
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

# Add the lidar sensor to the manager
manager.AddSensor(lidar)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Update the sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)