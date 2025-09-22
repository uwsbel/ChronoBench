import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math


def main():
    
    
    
    
    artcar = veh.ARTcar()
    artcar.SetContactMethod(chrono.ChContactMethod_NSC)
    artcar.SetChassisCollisionType(veh.CollisionType_NONE)
    artcar.SetChassisFixed(False)
    artcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
    artcar.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    artcar.SetDriveType(veh.DrivelineTypeWV_AWD)
    artcar.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    artcar.SetTireType(veh.TireModelType_TMEASY)
    artcar.SetTireStepSize(step_size)
    artcar.Initialize()

    artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    artcar.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    mphysicalSystem = artcar.GetSystem()

    
    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(10, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    
    
    driver = veh.ChInteractiveDriverIRR()

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                            chrono.CSYSNORM, 
                            200.0, 200.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),    
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
        artcar.GetChassisBody(),    
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 3), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        artcar.GetChassisBody(),    
        30,                         
        camera_offset_pose,         
        1280,                       
        720,                        
        chrono.CH_PI / 3           
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0)
    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    
    
    
    render_time = 0
    t1 = time.time()

    while mphysicalSystem.GetChTime() < end_time:
        time_now = mphysicalSystem.GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time_now)
        terrain.Synchronize(time_now)
        artcar.Synchronize(time_now, driver_inputs, terrain)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        artcar.Advance(step_size)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

    print("Sim time:", end_time, "Wall time:", time.time() - t1)







noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN


update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


render_step_size = 1.0/50  


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






if __name__ == "__main__":
    main()