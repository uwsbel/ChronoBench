import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.vehicle.artcar as artcar

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------
    # Create the ARTcar vehicle
    # ----------------------
    vehicle = artcar.ChVehicleARTcar()
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisMass(1000)
    vehicle.SetChassisInertiaXX(chrono.ChVector3d(100, 100, 100))
    vehicle.SetChassisCOM(chrono.ChVector3d(0, 0, 0.5))
    vehicle.SetChassisDimensions(chrono.ChVector3d(2.0, 1.5, 0.5))
    vehicle.SetWheelRadius(0.3)
    vehicle.SetWheelWidth(0.2)
    vehicle.SetWheelMass(10)
    vehicle.SetWheelInertia(0.1)
    vehicle.SetWheelSuspensionTravel(0.1)
    vehicle.SetWheelSuspensionStiffness(10000)
    vehicle.SetWheelSuspensionDamping(1000)
    vehicle.SetWheelTireStiffness(50000)
    vehicle.SetWheelTireDamping(500)
    vehicle.SetWheelTireFriction(0.8)

    # Initialize the vehicle at the specified location
    vehicle.Initialize(chrono.ChCoordinatorys(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))

    # Add the vehicle to the system
    mphysicalSystem.Add(vehicle.GetSystem())

    # -----------------------
    # Create a vehicle driver
    # -----------------------
    driver = veh.ChDriver()
    driver.SetSteering(0.0)
    driver.SetThrottle(0.5)
    driver.SetBraking(0.0)

    # ----------------------
    # Create the terrain
    # ----------------------
    terrain = veh.ChTerrain(mphysicalSystem)
    terrain.SetContactMaterialProperties(1e6, 0.8, 0.4)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetHeightField(0, 0, 100, 100, 0, 0)  # Flat terrain

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to the vehicle chassis
        update_rate,               # Scanning rate in Hz
        offset_pose,               # Offset pose
        horizontal_samples,        # Number of horizontal samples
        vertical_samples,          # Number of vertical channels
        horizontal_fov,            # Horizontal field of view
        max_vert_angle,            # Maximum vertical field of view
        min_vert_angle,            # Minimum vertical field of view
        100.0,                     # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,             # Sample radius
        divergence_angle,          # Divergence angle
        divergence_angle,          # Divergence angle (again, typically same value)
        return_mode                # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to the vehicle chassis
        update_rate,               # Scanning rate in Hz
        offset_pose,               # Offset pose
        horizontal_samples,        # Number of horizontal samples
        1,                         # only 1 vertical channel for 2D lidar
        horizontal_fov,            # Horizontal field of view
        0.0,                       # Maximum vertical field of view
        0.0,                       # Minimum vertical field of view
        100.0,                     # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,             # Sample radius
        divergence_angle,          # Divergence angle
        divergence_angle,          # Divergence angle (again, typically same value)
        return_mode                # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # -----------------------
    # Create a camera sensor
    # -----------------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-2.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        update_rate,               # Update rate in Hz
        camera_offset,             # Offset pose
        640,                       # Image width
        480,                       # Image height
        chrono.CH_PI / 3,          # Field of view
        100.0                      # Clipping far plane
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update the driver
        driver.SetThrottle(0.3)
        driver.SetSteering(0.1 * math.sin(ch_time * 0.5))

        # Synchronize and advance the vehicle
        vehicle.Synchronize(ch_time)
        vehicle.Update()

        # Update the terrain
        terrain.Synchronize(ch_time)

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()