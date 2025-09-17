import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, 0))  

    
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))  
    
    
    mesh_shape = chrono.ChTriangleMeshShape()
    mesh_shape.SetMesh(mesh)
    mesh_body.AddAsset(mesh_shape)
    system.Add(mesh_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)

    
    lidar_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))  
    horizontal_samples = 90
    vertical_samples = 45
    horizontal_fov = 360  
    vertical_fov = 40     
    lag = 0
    exposure_time = 0
    lens_model = sens.DualDepthModel()
    max_distance = 100

    
    lidar = sens.ChLidarSensor(
        mesh_body,            
        update_rate=10,       
        lidar_offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        vertical_fov,
        max_distance,
        lens_model,
        lag,
        exposure_time
    )
    lidar.SetName("Orbiting Lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))
    lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/lidar_%d.ply"))  
    manager.AddSensor(lidar)

    
    orbit_radius = 5.0
    orbit_height = 2.0
    orbit_speed = 0.1  
    time_step = 0.01
    total_time = 5.0

    
    time = 0.0
    while time < total_time:
        
        angle = orbit_speed * time
        x = orbit_radius * math.cos(angle)
        y = orbit_radius * math.sin(angle)
        lidar_offset_pose = chrono.ChFrameD(chrono.ChVectorD(x, y, orbit_height))
        lidar.SetOffsetPose(lidar_offset_pose)

        
        manager.Update()
        system.DoStepDynamics(time_step)
        time += time_step

        
        buffer = lidar.GetMostRecentBuffer()
        if buffer.HasData():
            di_buffer = sens.DepthFrameAccess(buffer)
            print(f"Time: {time:.2f}s | "
                  f"Points: {di_buffer.width * di_buffer.height} | "
                  f"Min Dist: {np.min(di_buffer.depth):.3f}m | "
                  f"Max Dist: {np.max(di_buffer.depth):.3f}m")

if __name__ == "__main__":
    main()