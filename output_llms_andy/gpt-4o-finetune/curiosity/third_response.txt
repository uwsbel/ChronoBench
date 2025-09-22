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

# Create the Irrlicht visualization
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


# Create a sensor manager
manager = sens.ChSensorManager(system)

# Set lidar position and orientation
lidar_update_rate = 5
horizontal_samples = 800
vertical_samples = 300
max_lidar_range = 30
lidar_horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
lidar_vertical_fov = chrono.CH_PI / 6  # 30 degrees
lidar_roll = 0
lidar_pitch = 0
lidar_yaw = 0
lidar_displacement = chrono.ChVector3d(0, 0, 1)

# Create a lidar sensor and attach it to the rover
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(rover.GetChassisBody(),              # Body lidar is attached to
                          lidar_update_rate,          # Scanning rate in Hz
                          offset_pose,                # Offset pose
                          horizontal_samples,         # Number of horizontal samples
                          vertical_samples,           # Number of vertical channels
                          max_lidar_range,            # Maximum radius (or detection distance)
                          lidar_horizontal_fov,       # Horizontal field of view
                          lidar_vertical_fov,         # Vertical field of view
                          -chrono.CH_PI / 12,         # Minimum vertical angle
                          2*chrono.CH_PI / 3,          # Maximum vertical angle
                          sens.LidarHorizontalVertical,  # Horizontal-Vertical sampling
                          1,                          # Sample radius (usually 1)
                          2)  # Lag time, in frames
lidar.SetName("Lidar Sensor")
lidar.SetCollectionWindow(1.0 / lidar_update_rate)
lidar.PulseModel.SetFrequency(5e5)  # 500 kHz
lidar.PulseModel.SetLongWaveInfrared(False)
lidar.PulseModel.SetDivergence(1e-3, 1e-3)  # 1 mrad divergence

lidar.AddFilter(sens.ChFilterDIAccess())
lidar.AddFilter(sens.ChFilterPCfromDepth())
lidar.AddFilter(sens.ChFilterVisualizePointCloud(640, 480,"Lidar Point Cloud"))
lidar.SetOffsetPose(offset_pose)
# Add the lidar sensor to the sensor manager
manager.AddSensor(lidar)
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