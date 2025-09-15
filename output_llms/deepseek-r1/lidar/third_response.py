import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # -----------------
    # Simulation parameters
    # -----------------
    step_size = 1e-3
    end_time = 40.0
    vis = True
    noise_model = "NONE"
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    lag = 0
    collection_time = 1.0 / update_rate
    sample_radius = 2
    divergence_angle = 0.003
    out_dir = "SENSOR_OUTPUT/"

    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Create and initialize ARTcar vehicle
    # ----------------------------------
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    chassis = vehicle.GetChassisBody()

    # -----------------
    # Create a driver
    # -----------------
    driver = veh.ChDriver(vehicle)

    # -----------------
    # Create terrain
    # -----------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        chassis,              # Body lidar is attached to (vehicle chassis)
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
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
        chassis,              # Body lidar is attached to (vehicle chassis)
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        1,                      # only 1 vertical channel for 2D lidar
        horizontal_fov,         # Horizontal field of view
        0.0,                    # Maximum vertical field of view
        0.0,                    # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
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
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Add third person camera attached to vehicle
    camera_pose = chrono.ChFramed(chrono.ChVector3d(2, 0, 1.5), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
    camera = sens.ChCameraSensor(
        chassis,             # Body camera is attached to
        30,                  # Update rate in Hz
        camera_pose,         # Offset pose
        1280,                # Image width
        720,                 # Image height
        chrono.CH_PI / 3     # Horizontal field of view
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0.1)
    camera.SetCollectionWindow(0.02)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update vehicle systems
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain)
        terrain.Synchronize(ch_time)
        
        # Update sensor manager
        manager.Update()

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer and xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Advance simulation
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        manager.Advance(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()