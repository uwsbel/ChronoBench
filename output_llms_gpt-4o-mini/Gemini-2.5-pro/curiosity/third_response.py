import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens # 1. Added sensor module import

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9) # Example: set friction for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# create a long box for rover to cross
box_mat = chrono.ChContactMaterialNSC() # Can use same ground_mat or a new one
box_mat.SetFriction(0.7) # Example: set friction for box
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, box_mat)
# Corrected Z position: box height is 0.25, so center at 0.125 for bottom to be at z=0
box.SetPos(chrono.ChVector3d(0, 0, 0.125))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
# Rover uses its own default NSC material if not specified
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
# Corrected Z position: Start rover higher to avoid ground penetration
init_pos = chrono.ChVector3d(-5, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# 2. Create a sensor manager
manager = sens.ChSensorManager(system)
manager.SetVerbose(False) # Optional: reduce console output from sensor manager

# 3. Add a lidar sensor to the rover
update_rate = 10  # Hz (reduced for performance in typical demos)
horizontal_samples = 180 # Reduced for performance
vertical_samples = 16    # Reduced for performance
horizontal_fov = math.pi   # 180 degrees (e.g. forward scan)
# Vertical FOV: total angle, e.g., 30 degrees -> +/- 15 deg from center
vertical_fov_total = math.pi / 6  # 30 degrees
max_distance = 100.0
lag = 0.0 # LIDAR lag (s)
exposure_time = 0.0 # LIDAR exposure time (s)

# Lidar pose relative to chassis (1m forward, 0.5m up, facing forward)
# Rover chassis X is forward, Y is left, Z is up
offset_pose = chrono.ChFrameD(chrono.ChVector3d(1.0, 0, 0.5), 
                              chrono.ChQuaterniond(1, 0, 0, 0))

lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  # Parent body (rover chassis)
    update_rate,             # Update rate in Hz
    offset_pose,             # Offset pose relative to parent
    horizontal_samples,      # Number of horizontal samples
    vertical_samples,        # Number of vertical samples
    horizontal_fov,          # Horizontal field of view
    vertical_fov_total,      # Total vertical field of view (symmetric)
    max_distance             # Max detection distance
    # Optional: lag, exposure_time, beam_shape, sample_radius, noise_stddev, return_mode, clip_near
)
lidar.SetName("Lidar Sensor")

# Add filters for lidar data processing
# Filter to create a point cloud from raw depth data
lidar.PushFilter(sens.ChFilterPCfromDepth())
# Filter to visualize the generated point cloud (optional)
# Parameters: width, height, point_size, window_name
lidar.PushFilter(sens.ChFilterVisualizePointCloud(800, 600, 0.05, "Lidar Point Cloud"))

# Add the lidar sensor to the sensor manager
manager.AddSensor(lidar)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 6, 3), chrono.ChVector3d(0, 0, 0.5)) # Adjusted camera for better view
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)
# vis.EnableShadows() # Can be enabled, may impact performance

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    current_time = system.GetChTime()
    # time += time_step # Using system time is often more robust

    # Ask rover to move forward
    driver.SetSteering(0.0)
    driver.SetThrottle(0.5) # Corrected: Added throttle command

    # Update rover dynamics
    rover.Update()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)

    # 4. Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    # chrono.ChronoEngine_ zowel Irrlicht als Sensor hebben Render functies.
    # The Lidar visualization filter has its own rendering window.
    vis.EndScene()


    if current_time > 20: # Example: Stop simulation after 20 seconds
        vis.GetDevice().closeDevice()


vis.EndLoop() # Proper way to end Irrlicht visualization if loop broken by other means

print("Simulation finished.")