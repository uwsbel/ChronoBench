import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import os
import math

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, 0))

    
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))  
    mesh_shape = chrono.ChTriangleMeshShape(mesh)
    mesh_shape.SetStatic(True)
    mesh_body.AddAsset(mesh_shape)
    system.Add(mesh_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2, 2), chrono.ChColor(1, 1, 1), 100)

    
    lidar_offset = chrono.ChVectorD(0, 0.5, 0)  
    lidar_pos = chrono.ChFrameD(lidar_offset)    
    lidar_update_rate = 5                        
    lidar_hres = 90                              
    lidar_vres = 45                              
    lidar_hfov = 180                             
    lidar_vfov = 30                              

    
    noise_model = sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0.01), chrono.ChVectorD(0, 0, 0.001))
    lidar = sens.ChLidarSensor(
        mesh_body,                
        lidar_update_rate,        
        lidar_pos,                
        lidar_hres,               
        lidar_vres,               
        math.radians(lidar_hfov), 
        math.radians(lidar_vfov), 
        0.1,                      
        sens.ClipFrustum()        
    )
    lidar.SetName("Rotating Lidar")
    lidar.SetLag(0.1)
    lidar.SetCollectionWindow(0.02)
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterAccess())
    lidar.PushFilter(sens.ChFilterVisualize(lidar_hres, lidar_vres, "Lidar Data"))
    lidar.PushFilter(sens.ChFilterSave("lidar_data/"))
    lidar.PushFilter(sens.ChFilterNoise(noise_model))
    manager.AddSensor(lidar)

    
    orbit_radius = 3.0
    orbit_height = 1.0
    orbit_speed = 0.1  
    time_step = 0.01
    total_time = 10.0

    for t in np.arange(0, total_time, time_step):
        
        angle = orbit_speed * t
        new_x = orbit_radius * math.cos(angle)
        new_z = orbit_radius * math.sin(angle)
        lidar_offset = chrono.ChVectorD(new_x, orbit_height, new_z)
        lidar.SetOffsetPose(chrono.ChFrameD(lidar_offset))

        
        manager.Update()
        system.DoStepDynamics(time_step)

        
        buffer = lidar.GetMostRecentBuffer()
        if buffer.HasData():
            data = buffer.GetDepthData()
            print(f"Time: {t:.2f}s | Points: {len(data)}")
            if len(data) > 0:
                print(f"First point: Pos={data[0]} | Intensity={data[0].intensity}")

if __name__ == "__main__":
    main()