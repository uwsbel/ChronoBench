import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"  # No noise model
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation

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


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Create ARTcar vehicle
    # ----------------------------------
    # Set the path to the Chrono vehicle data directory
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")
    
    # Create the vehicle system (ARTcar)
    vehicle_type = "ARTcar"
    
    # Vehicle parameters
    chassis_mass = 250.0
    chassis_cm_position = chrono.ChVector3d(0, 0.5, 0)
    chassis_inertia = chrono.ChMatrix3d()
    chassis_inertia.SetIdentity()
    
    # Create the vehicle system
    vehicle = veh.ChVehicleArticulation(vehicle_type, mphysicalSystem)
    
    # Set driver (default driver)
    driver = veh.ChDriver(vehicle.GetChassis())
    
    # ----------------------------------
    # Add terrain
    # ----------------------------------
    # Create terrain material
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.9)
    terrain_material.SetRestitution(0.1)
    
    # Create the terrain
    terrain = chrono.ChTerrain(mphysicalSystem)
    terrain.Initialize(terrain_material, 100.0, 100.0, 0.0)
    
    # Set terrain texture and color
    terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    
    # ----------------------------------
    # Create a sensor manager
    # ----------------------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    # Get the vehicle chassis body for attaching sensors
    chassis = vehicle.GetChassis()
    
    # Changed Lidar Offset Pose from (-12, 0, 1) to (1.0, 0, 1)
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        chassis,                  # Body lidar is attached to (changed from box to chassis)
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
        divergence_angle          # Divergence angle (fixed - removed duplicate)
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
    
    # Provides the host access to the Depth, Intensity data (must be before visualization)
    lidar.PushFilter(sens.ChFilterDIAccess())
    
    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    
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
        chassis,                  # Body lidar is attached to (changed from box to chassis)
        update_rate,              # Scanning rate in Hz
        offset_pose,              # Offset pose
        horizontal_samples,       # Number of horizontal samples
        1,                        # only 1 vertical channel for 2D lidar
        horizontal_fov,           # Horizontal field of view
        0.0,                      # Maximum vertical field of view
        0.0,                      # Minimum vertical field of view
        100.0,                    # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,            # Sample radius
        divergence_angle          # Divergence angle (fixed - removed duplicate)
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    
    # Provides the host access to the Depth, Intensity data (must be before visualization)
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)
    
    # ----------------------------------
    # Add Third Person Camera
    # ----------------------------------
    # Camera offset pose - positioned behind and above the vehicle
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-5.0, 2.0, 0),  # Behind and above the chassis
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    camera = sens.ChCameraSensor(
        chassis,                  # Body camera is attached to
        update_rate,              # Update rate in Hz
        camera_offset_pose,       # Offset pose
        640,                      # Image width
        480,                      # Image height
        1.0,                      # Horizontal field of view (radians)
        1                          # Number of samples per pixel (for super sampling)
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    
    # Add camera filters
    # Provides host access to RGBA buffer
    camera.PushFilter(sens.ChFilterRGBAAccess())
    
    if vis:
        # Visualize the camera feed
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person View"))
    
    # Add camera to sensor manager
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
        # Set lidar to orbit around the mesh body (updated to use chassis position)
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Also update 2D lidar pose
        lidar_2d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Synchronize driver input
        driver.Synchronize(ch_time)
        
        # Synchronize vehicle
        vehicle.Synchronize(ch_time)
        
        # Synchronize terrain (if needed)
        terrain.Synchronize(ch_time)

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Advance vehicle state
        vehicle.Advance(step_size)
        
        # Advance driver state
        driver.Advance(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()