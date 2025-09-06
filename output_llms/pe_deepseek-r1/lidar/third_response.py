import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()
    
    # -------------------------
    # Create and initialize ARTcar
    # -------------------------
    contact_method = chrono.ChContactMethod_NSC
    chassis_collision_type = veh.CollisionType_NONE
    initLoc = chrono.ChVector3d(0, 0, 0.5)
    initRot = chrono.QUNIT
    
    car = veh.ARTcar()
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.Initialize()
    
    # Set visualization type for vehicle components
    car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # ---------------------
    # Create vehicle driver
    # ---------------------
    driver = veh.ChDriver(car.GetVehicle())
    
    # ---------------------
    # Create rigid terrain
    # ---------------------
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
        chrono.ChVector3d(1.0, 0, 1),  # Changed offset position
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),    # Attached to vehicle chassis
        update_rate,             
        offset_pose,             
        horizontal_samples,      
        vertical_samples,        
        horizontal_fov,          
        max_vert_angle,          
        min_vert_angle,          
        100.0,                   
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,           
        divergence_angle,        
        divergence_angle,        
        return_mode              
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
        pass
        
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
        
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
        
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(),    # Attached to vehicle chassis
        update_rate,             
        offset_pose,             
        horizontal_samples,      
        1,                       # Single vertical channel
        horizontal_fov,          
        0.0,                     
        0.0,                     
        100.0,                   
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,           
        divergence_angle,        
        divergence_angle,        
        return_mode              
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
    
    # ----------------------------------------
    # Add third-person camera to vehicle
    # ----------------------------------------
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-3, 0, 2),  # Position behind and above vehicle
        chrono.QuatFromAngleAxis(0.3, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),  # Attached to vehicle chassis
        30,                    # Update rate
        camera_offset_pose,    # Offset pose
        1280,                  # Image width
        720,                   # Image height
        chrono.CH_PI / 3       # HFOV (60 degrees)
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
        time = mphysicalSystem.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        
        # Synchronize modules
        driver.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        # Update sensor manager
        manager.Update()
        
        # Advance simulation
        driver.Advance(step_size)
        car.Advance(step_size)
        terrain.Advance(step_size)
        mphysicalSystem.DoStepDynamics(step_size)
        
        # Get current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Lidar parameters
# -----------------
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

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

main()