import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
# Import the sensor module
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

# ------- Sensor setup -------
# Create a sensor manager
manager = sens.ChSensorManager(system)

# Define lidar parameters
update_rate = 30  # Hz
h_samples = 180
v_samples = 16
fov_h = chrono.CH_C_PI  # 180 degrees
fov_v = chrono.CH_C_PI / 4  # 45 degrees
max_distance = 5.0
# Create the lidar sensor
lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  # Attach to rover chassis
    update_rate,             # Update rate
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.2)),  # Sensor frame relative to body
    h_samples,               # Horizontal samples
    v_samples,               # Vertical samples
    fov_h,                   # Horizontal FOV
    fov_v,                   # Vertical FOV
    max_distance,           # Max sensing distance
)
lidar.SetIntensityMode(False)  # Disable intensity output

# Set up filters for the lidar data (optional)
# ...

# Add the lidar sensor to the sensor manager
manager.AddSensor(lidar)

# ------- End of sensor setup -------

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

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
    driver.SetSteering(0.0)
    driver.SetMotorSpeeds(1.0, 1.0)  # Assuming two motors, set both to move forward

    # Update rover dynamics
    rover.Update()

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)