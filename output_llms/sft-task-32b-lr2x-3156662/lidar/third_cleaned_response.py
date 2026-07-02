import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.visualization as vis
import numpy as np
import time
import math

def main():
    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    patch.SetTexture(chrono.GetChronoDataFile("textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    driver = veh.ChDriver(vehicle)
    driver.Initialize()
    
    
    
    
    manager = sens.ChSensorManager(vehicle.GetSystem())
    
    
    
    
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
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    noise_model = "NONE"
    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    
    
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  
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
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if True:  
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if True:  
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)
    
    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  
        update_rate,
        offset_pose,
        horizontal_samples,
        1,  
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
    if True:  
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)
    
    
    
    
    camera_offset = chrono.ChFramed(chrono.ChVector3d(1.5, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        30.0,  
        camera_offset,
        1280,  
        720,   
        1.5708  
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    manager.AddSensor(camera)
    
    
    
    
    step_size = 1e-3
    end_time = 40.0
    vis = True
    
    while vehicle.GetSystem().GetChTime() < end_time:
        
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = 0.0
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_braking = 0.0
        driver.Synchronize(vehicle.GetSystem().GetChTime())
        driver.Advance(step_size)
        
        
        vehicle.Synchronize(vehicle.GetSystem().GetChTime(), driver_inputs, terrain)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        
        
        manager.Update()
        
        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer and xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f"XYZI buffer received from lidar. Lidar resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}")
            print(f"Max Value: {np.max(xyzi_data)}")
        
        
        vehicle.GetSystem().DoStepDynamics(step_size)
    
    print(f"Sim time: {end_time} seconds")

main()