import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens
from matplotlib import pyplot as plt


def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    body_truss = chrono.ChBody()
    body_truss.setFixed(True)
    mphysicalSystem.add(body_truss)

    
    
    
    
    trimesh = chrono.ChTriangleMeshConnected()
    trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    trimesh.Transform(chrono.ChVector3d(-1.5, 0, 0), chrono.ChMatrix33d( 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5))

    
    
    
    trimesh_shape = chrono.ChVisualShapeTriangleMesh(trimesh)
    trimesh_shape.SetName("HMMWV CHASSIS MESH")
    trimesh_shape.SetMutable(False)  
    body_truss.AddVisualShape(trimesh_shape)  

    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    update_rate = 15.0
    manager.SetUpdateRate(update_rate)

    
    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    
    lidar = sens.ChLidarSensor(body_truss,              
                               update_rate,              
                               offset_pose,              
                               100,                      
                               3.1415926,                 
                               50,                       
                               chrono.CH_PI / 12,        
                               chrono.CH_PI / 24,        
                               50.0,                     
                               sens.LidarBeamShape_RECTANGULAR,  
                               1.0,                      
                               2.0 / 1000.0               
                               )
    lidar.SetName("Lidar Sensor")
    
    

    
    
    
    
    
    lidar.PushFilter(sens.ChFilterLidarReturn())
    
    lidar.PushFilter(sens.ChFilterDDA466Noise())
    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Point Cloud"))

    
    lidar.PushFilter(sens.ChFilterSave("../SENSOR_OUTPUT/SCENE_LIDAR/"))

    
    lidar.PushFilter(sens.ChFilterOnFile("../SENSOR_OUTPUT/PCD_DATA/lidar_data.pcd", "%04.0f"))  
    
    access_buffer = sens.ChFilterAccess()
    lidar.PushFilter(access_buffer)

    
    manager.AttachSensor(lidar)

    
    
    
    orbit_radius = 3
    orbit_rate = 0.5
    time = 0
    time_orbit = 2 * 3.1415926 / orbit_rate
    while True:
        time += step_size
        
        new_lidar_pose = chrono.ChFramed(
            chrono.ChVector3d(orbit_radius * math.cos(orbit_rate * time), orbit_radius * math.sin(orbit_rate * time), 1),
            chrono.QuatLookAt(
                (chrono.ChVector3d(orbit_radius * math.cos(orbit_rate * time), orbit_radius * math.sin(orbit_rate * time), 1)),
                (chrono.ChVector3d(0, 0, 1))))
        lidar.SetOffsetPose(new_lidar_pose)

        
        buffer = access_buffer.GetPCfreeBuffer()
        print(f"Lidar buffer points: {buffer.GetPoints().shape[0]}")

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()