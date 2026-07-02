import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    vehicle = chrono.ChVehicleArticulatedTracked()
    vehicle.Initialize(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0), True)
    vehicle.SetMass(2000)
    vehicle.SetInertiaXX(chrono.ChVector3d(10000, 10000, 10000))
    vehicle.SetInertiaXY(chrono.ChVector3d(0, 0, 0))
    mphysicalSystem = vehicle.GetSystem()

    # ---------------------
    # Create a rigid terrain
    # ---------------------
    terrain = chrono.ChSystemNSC()
    terrain.AddTerrain(chrono.ChTerrainRigidPlane(chrono.ChVector3d(0, 0, -1), chrono.ChVector3d(0, 0, 1)))
    terrain.GetTerrain(0).SetMaterialProperties(0.01, 0.01, 0.01)
    terrain.GetTerrain(0).SetTexture(chrono.GetChronoDataFile("textures/ground.jpg"))
    terrain.GetTerrain(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    mphysicalSystem.Add(terrain)

    # -----------------
    # Create a driver
    # -----------------
    driver = chrono.ChDriver()
    driver.SetVehicle(vehicle)
    driver.SetThrottle(0.2)
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # -----------------------
    # Create a lidar sensor
    # -----------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        update_rate,              # Scanning rate in Hz
        offset_pose,              # Offset pose
        horizontal_samples,       # Number of horizontal samples
        vertical_samples,         # Number of vertical channels
        horizontal_fov,           # Horizontal field of view
        max_vert_angle,           # Maximum vertical field of view
        min_vert_angle,           # Minimum vertical field of view
        100.0,                    # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,            # Sample radius
        divergence_angle,         # Divergence angle
        divergence_angle,         # Divergence angle (again, typically same value)
        return_mode               # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # -----------------------
    # Create a 2D lidar sensor
    # -----------------------
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
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
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # ---------------------
    # Create a third-person camera
    # ---------------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1.5), chrono.QuatFromAngleAxis(-0.5, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        update_rate,               # Scanning rate in Hz
        camera_offset,             # Offset pose
        640,                       # Image width
        480,                       # Image height
        0.0,                       # Near clipping plane
        100.0                      # Far clipping plane
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person Camera"))
    manager.AddSensor(camera)

    # -------------------
    # Simulate system
    # -------------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update driver
        driver.Update(ch_time)

        # Update vehicle
        vehicle.Advance(ch_time)

        # Update terrain
        terrain.Advance(ch_time)

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
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

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