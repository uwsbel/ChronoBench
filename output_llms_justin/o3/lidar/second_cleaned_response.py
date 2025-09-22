import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math







noise_model        = "NONE"                         
return_mode        = sens.LidarReturnMode_STRONGEST_RETURN
update_rate        = 5.0                            

horizontal_samples = 800                            
vertical_samples   = 300                            

horizontal_fov     = 2 * chrono.CH_PI              
max_vert_angle     =  chrono.CH_PI / 12.0          
min_vert_angle     = -chrono.CH_PI /  6.0          

lag                = 0.0
collection_time    = 1.0 / update_rate
sample_radius      = 2
divergence_angle   = 0.003                          


vertical_samples_2d = 1                             


step_size          = 1e-3
end_time           = 40.0

vis                = True
out_dir            = "SENSOR_OUTPUT/"
save               = False      



def build_lidar(parent_body,
                offset_pose,
                h_samples,
                v_samples,
                name_suffix=""):

    

    lidar = sens.ChLidarSensor(
        parent_body,                 
        update_rate,                 
        offset_pose,                 
        h_samples,                   
        v_samples,                   
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

    lidar.SetName(f"Lidar Sensor{name_suffix}")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualize(
                h_samples, v_samples, f"Raw Lidar Depth Data{name_suffix}"
            )
        )

    
    lidar.PushFilter(sens.ChFilterDIAccess())

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640, 480, 1.0, f"Lidar Point Cloud{name_suffix}"
            )
        )

    
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    return lidar


def main():
    
    
    
    sys = chrono.ChSystemNSC()

    
    
    
    side      = 2.0                                        
    density   = 1000                                       

    cube_body = chrono.ChBodyEasyBox(side, side, side,     
                                     density,
                                     True,  True)          

    cube_body.SetPos(chrono.ChVector3d(0, 0, 0))
    cube_body.SetFixed(True)

    
    texture = chrono.ChVisualShapeTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    cube_body.AddVisualShape(texture)

    sys.Add(cube_body)

    
    
    
    manager = sens.ChSensorManager(sys)

    
    manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 500)

    
    
    
    
    offset_pose_3d = chrono.ChFrameD(
        chrono.ChVector3d(-12, 0, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)                   
    )

    
    lidar_3d = build_lidar(cube_body,
                           offset_pose_3d,
                           horizontal_samples,
                           vertical_samples,
                           "_3D")

    manager.AddSensor(lidar_3d)

    
    
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVector3d(-12, 0, 1.5),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = build_lidar(cube_body,
                           offset_pose_2d,
                           horizontal_samples,
                           vertical_samples_2d,
                           "_2D")

    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10.0
    orbit_rate   = 0.1

    render_time  = 0.0
    t_wall_start = time.time()

    while sys.GetChTime() < end_time:

        ch_time = sys.GetChTime()

        
        new_offset = chrono.ChFrameD(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1.0
            ),
            chrono.ChQuaternionD(                             
                chrono.ChRotationD(chrono.Q_from_AngAxis(ch_time * orbit_rate,
                                                          chrono.ChVector3d(0, 0, 1)))
            )
        )
        lidar_3d.SetOffsetPose(new_offset)

        

        
        manager.Update()

        
        xyzi_buf = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buf.HasData():
            xyzi_data = xyzi_buf.GetXYZIData()
            print(f"[{ch_time:6.2f}s] 3-D lidar  XYZI received – "
                  f"resolution {xyzi_buf.Width}×{xyzi_buf.Height}  "
                  f"max-val {np.max(xyzi_data):.4f}")

        
        sys.DoStepDynamics(step_size)

    print("Simulation time: {:.2f} s   |   Wall-clock time: {:.2f} s"
          .format(end_time, time.time() - t_wall_start))





if __name__ == "__main__":
    main()