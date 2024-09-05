import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math








vehicle_system = veh.ChVehicleSystemNSC()


vehicle = veh.ChVehicle("My ARTcar", vehicle_system)


vehicle.SetChassisFixed(False)
vehicle.SetInitWheelPos(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
vehicle.SetSteeringSharpen(1.0)
vehicle.SetTireType(veh.ChVehicle.TIRE_MODEL_RIGID)


driver = veh.ChDriver(vehicle)


terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000)
terrain.SetPos(chrono.ChVector3d(0, 0, -0.05))
terrain.SetBodyFixed(True)
terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
vehicle_system.Add(terrain)








manager = sens.ChSensorManager(vehicle_system)




offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
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
elif noise_model == "NONE":
    
    pass
if vis:
    
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))

lidar_2d.PushFilter(sens.ChFilterDIAccess())

lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

lidar_2d.
print("error happened with only start ```python")