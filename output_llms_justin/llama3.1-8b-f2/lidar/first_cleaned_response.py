import pychrono as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    
    
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "models/obj/porsche911/porsche911.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Porsche 911 Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        mesh_body,              
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
        return_loss            
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, vertical_samples, "Lidar Depth Data"))

    
    if save:
        lidar.PushFilter(sens.ChFilterSave(out_dir + "lidar/"))

    
    if vis_pcl:
        lidar.PushFilter(sens.ChFilterToPointCloud(
            point_cloud_radius,  
            100  
        ))
        lidar.PushFilter(sens.ChFilterPCLVisualize(100, 100, "Lidar Point Cloud"))

    
    lidar.ComposeFilterGraph()

    
    
    
    manager.AddSensor(lidar)

    
    
    
    orbit_radius = 5
    orbit_rate = 0.2
    ch_time = 0.0

    t1 = time.time()

    while (ch_time < end_time):
        lidar.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -
                             orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        
        lidar_buffer = lidar.GetMostRecentBuffer()
        if (lidar_buffer.HasData()):
            data = lidar_buffer.GetData()
            print('Lidar buffer, max: ', np.max(data), ' min: ', np.min(data))
        else:
            print('Lidar buffer is empty\n')

        
        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)









noise_model = "NONE"                  


update_rate = 5


horizontal_samples = 1000
vertical_samples = 20


horizontal_fov =  chrono.CH_PI / 9  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 0


divergence_angle = 0.003


return_loss = False






step_size = 1e-3


end_time = 20.0


save = False


vis_pcl = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()