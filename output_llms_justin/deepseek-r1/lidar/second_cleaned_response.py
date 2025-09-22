import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math






side = 2.0


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


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"

def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.GetVisualShape(0).SetTexture(texture)
    
    mphysicalSystem.Add(box_body)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    offset_pose_3d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,              
        update_rate,           
        offset_pose_3d,        
        horizontal_samples,    
        vertical_samples,      
        horizontal_fov,        
        max_vert_angle,        
        min_vert_angle,        
        100.0,                 
        sens.BeamShape_RECTANGULAR,  
        sample_radius,         
        divergence_angle,      
        divergence_angle,      
        return_mode            
    )
    lidar_3d.SetName("3D Lidar")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_3d)

    
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,              
        update_rate,           
        offset_pose_2d,        
        horizontal_samples,    
        1,                     
        horizontal_fov,        
        0.001,                 
        0,                     
        100.0,                 
        sens.BeamShape_RECTANGULAR,  
        sample_radius,         
        divergence_angle,      
        divergence_angle,      
        return_mode            
    )
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar_3d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f'3D Lidar: {xyzi_buffer.Width}x{xyzi_buffer.Height} points | Max intensity: {np.max(xyzi_data["intensity"]):.2f}')

        
        manager.Update()
        
        
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulation time: {end_time}s | Wall time: {time.time()-t1:.2f}s")

if __name__ == "__main__":
    main()