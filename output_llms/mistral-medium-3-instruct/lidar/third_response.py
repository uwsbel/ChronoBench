import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irrl
import pychrono.assets as assets

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------
    # Create and initialize vehicle
    # ----------------------------
    vehicle = veh.ARTcar(mphysicalSystem)
    vehicle.Initialize()

    # -----------------------
    # Create vehicle driver
    # -----------------------
    driver = veh.ChDriver()
    driver.Initialize()

    # -----------------------
    # Create the terrain
    # -----------------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    terrain.SetContactFrictionCoefficient(0.8)
    terrain.SetContactRestitutionCoefficient(0.1)
    terrain.SetContactMaterialProperties(2e7, 0.3)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize(0, 0, 0)

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
        vehicle.GetChassis(),  # Body lidar is attached to (changed to vehicle chassis)
        update_rate,           # Scanning rate in Hz
        offset_pose,           # Offset pose
        horizontal_samples,    # Number of horizontal samples
        vertical_samples,      # Number of vertical channels
        horizontal_fov,        # Horizontal field of view
        max_vert_angle,        # Maximum vertical field of view
        min_vert_angle,        # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angle
        divergence_angle,      # Divergence angle (again, typically same value)
        return_mode            # Return mode for the lidar
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
        vehicle.GetChassis(),  # Body lidar is attached to (changed to vehicle chassis)
        update_rate,           # Scanning rate in Hz
        offset_pose,           # Offset pose
        horizontal_samples,    # Number of horizontal samples
        1,                     # only 1 vertical channel for 2D lidar
        horizontal_fov,        # Horizontal field of view
        0.0,                   # Maximum vertical field of view
        0.0,                   # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angle
        divergence_angle,      # Divergence angle (again, typically same value)
        return_mode            # Return mode for the lidar
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

    # ----------------------------
    # Add third person camera
    # ----------------------------
    camera = sens.ChCameraSensor(
        vehicle.GetChassis(),  # Body camera is attached to
        update_rate,           # Update rate in Hz
        chrono.ChFramed(chrono.ChVector3d(0, 0, 1.75), chrono.Q_from_AngZ(chrono.CH_PI)),  # Offset pose
        1280,                  # Image width
        720,                   # Image height
        1.0                    # Collection time
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    # Create Irrlicht application for visualization
    app = irrl.ChIrrApp(mphysicalSystem, 'Vehicle with Lidar', irrl.dimension2d(1280, 720))
    app.AddTypicalLogo()
    app.AddTypicalSky()
    app.AddTypicalLights()
    app.AddTypicalCamera(irrl.vector3df(0, 0, 2))
    app.AssetBindAll()
    app.AssetUpdateAll()

    while ch_time < end_time:
        # Set lidar to orbit around the vehicle
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

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Update vehicle and driver
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs())

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Update visualization
        app.BeginScene()
        app.DrawAll()
        app.DoStep()
        app.EndScene()

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
chrono.SetChronoDataPath('path/to/data')  # Update this path as needed

main()