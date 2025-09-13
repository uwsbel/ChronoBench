import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # -----------------
    # Simulation parameters
    # -----------------
    noise_model = "NONE"  # Noise model ("NONE" or "CONST_NORMAL_XYZI")
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    update_rate = 5.0  # Hz
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    lag = 0
    collection_time = 1.0 / update_rate
    sample_radius = 2
    divergence_angle = 0.003
    step_size = 1e-3
    end_time = 40.0
    vis = True
    out_dir = "SENSOR_OUTPUT/"

    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------
    # Create ARTcar vehicle
    # -----------------
    # Initialize vehicle parameters
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(1, 0, 0)))
    vehicle.Initialize()

    # Set vehicle visualization
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # Add vehicle to system
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    mphysicalSystem = vehicle.GetSystem()

    # -----------------
    # Create vehicle driver
    # -----------------
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    # -----------------
    # Create rigid terrain
    # -----------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
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
    # Attach to vehicle chassis with new offset position
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  # Changed offset to (1.0, 0, 1)
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),   # Attached to vehicle chassis
        update_rate,                # Scanning rate in Hz
        offset_pose,                # Offset pose
        horizontal_samples,         # Number of horizontal samples
        vertical_samples,           # Number of vertical channels
        horizontal_fov,             # Horizontal field of view
        max_vert_angle,             # Maximum vertical field of view
        min_vert_angle,             # Minimum vertical field of view
        100.0,                      # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius,              # Sample radius
        divergence_angle,           # Divergence angle
        divergence_angle,           # Vertical divergence angle
        return_mode                 # Return mode
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
        pass  # No noise model
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())  # Depth+intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())  # Convert to point cloud
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())  # XYZI data access
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),   # Attached to vehicle chassis
        update_rate,                # Scanning rate in Hz
        offset_pose,                # Offset pose
        horizontal_samples,         # Number of horizontal samples
        1,                          # Single vertical channel
        horizontal_fov,             # Horizontal field of view
        0.0,                        # Max vertical angle
        0.0,                        # Min vertical angle
        100.0,                      # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius,              # Sample radius
        divergence_angle,           # Divergence angle
        divergence_angle,           # Vertical divergence angle
        return_mode                 # Return mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass  # No noise model
    
    if vis:
        # Corrected visualization dimensions for 2D lidar
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)
    
    # -------------------------------
    # Add third-person camera sensor
    # -------------------------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 3),  # Position behind and above vehicle
        chrono.Q_from_AngAxis(0.3, chrono.ChVector3d(0, 1, 0))  # Slight downward angle
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Attached to vehicle
        30,                        # Update rate (Hz)
        camera_offset,             # Offset position
        1280,                      # Image width
        720,                       # Image height
        chrono.CH_PI / 3           # HFOV (60 degrees)
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
        # Update vehicle and driver systems
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain)
        terrain.Synchronize(ch_time)
        
        # Update sensor manager
        manager.Update()
        
        # Advance vehicle and terrain systems
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        
        # Access sensor data (example for lidar)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f'Lidar point count: {len(xyzi_data)} | Max Z: {np.max(xyzi_data[:,2]):.2f}')

        # Get current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Simulation time:", end_time, "seconds")
    print("Wall time:", time.time() - t1, "seconds")

if __name__ == "__main__":
    main()