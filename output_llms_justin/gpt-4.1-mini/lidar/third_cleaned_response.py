import pychrono.core as chrono
import pychrono.sensor as sens

import pychrono.vehicle as veh
import pychrono.vehicle.terrain as veh_terrain
import pychrono.vehicle.driver as veh_driver

import numpy as np
import time
import math
import os


def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

    vehicle = veh.ArticulatedVehicle()
    vehicle.SetContactMethod(chrono.ChMaterialSurfaceSMC.ContactMethod.SMC)
    vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(step_size)
    vehicle.Initialize()

    
    mphysicalSystem = vehicle.GetSystem()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    terrain = veh_terrain.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch_mat.SetPoissonRatio(0.3)
    patch_mat.SetSlipCompliance(1e-5)
    patch_mat.SetRollingFriction(0.02)
    patch_mat.SetSpinningFriction(0.02)

    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChVectorD(0, 0, 0), 
                             chrono.ChVectorD(0, 0, 1), 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), 200, 200)
    terrain.Initialize()

    
    
    
    driver = veh.ChWillemsDriver(vehicle)
    driver.Initialize()

    
    
    
    

    
    chassis = vehicle.GetChassisBody()

    
    offset_pose_3d = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar = sens.ChLidarSensor(
        chassis,                  
        update_rate,              
        offset_pose_3d,           
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

    
    
    
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = sens.ChLidarSensor(
        chassis,                
        update_rate,            
        offset_pose_2d,         
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
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    
    third_person_offset = chrono.ChFrameD(
        chrono.ChVectorD(-6, 0, 3),  
        chrono.ChQuaternionD(chrono.ChVectorD(0, 0, 1), math.radians(180))  
    )
    camera = sens.ChCameraSensor(
        chassis,
        30,                         
        third_person_offset,
        640,                        
        480,                        
        math.radians(60)            
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person Camera"))
    if save:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        camera.PushFilter(sens.ChFilterSave(out_dir))
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10  
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        
        
        
        
        

        

        time_step = step_size

        driver_inputs = driver.GetInputs()
        driver.Synchronize(ch_time)
        driver.Advance(time_step)

        terrain.Synchronize(ch_time)
        terrain.Advance(time_step)

        vehicle.Synchronize(ch_time, driver_inputs, terrain)
        vehicle.Advance(time_step)

        manager.Update()

        
        mphysicalSystem.DoStepDynamics(time_step)

        ch_time = mphysicalSystem.GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                f"Time {ch_time:.3f} s - 3D Lidar buffer received. Resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}"
            )
            print(f"Max Value: {np.max(xyzi_data):.3f}")

    print("Sim time:", end_time, "Wall time:", time.time() - t1)









noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_C_PI  
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6


lag = 0


collection_time = 1.0 / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"

if __name__ == "__main__":
    main()