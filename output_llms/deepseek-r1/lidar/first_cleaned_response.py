import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, 0))  

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    
    mesh_path = chrono.GetChronoDataFile("models/cube.obj")  
    mesh = chrono.ChVisualShapeModelFile()
    mesh.SetFilename(mesh_path)
    
    mesh_body = chrono.ChBody()
    mesh_body.AddVisualShape(mesh)
    mesh_body.SetFixed(True)  
    system.Add(mesh_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 100)

    
    lidar_offset = chrono.ChVector3d(0, 0, 0)
    lidar_rot = chrono.QuatFromAngleZ(chrono.CH_PI)  
    
    lidar = sens.ChLidarSensor(
        mesh_body,             
        10,                    
        lidar_rot,             
        1000,                  
        500,                   
        math.radians(90),      
        math.radians(30),      
        0.1,                   
        100.0                  
    )
    
    lidar.SetOffsetPose(lidar_offset)
    lidar.SetName("Lidar Sensor")

    
    noise_model = sens.ChNoiseNormal(chrono.ChVector3d(0.01, 0.01, 0.01))
    lidar.AddNoiseModel(noise_model)

    
    lidar.PushFilter(sens.ChFilterVisualize(1280, 720, "Lidar Data"))

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterSave("sensor_data/"))

    
    manager.AddSensor(lidar)

    
    orbit_radius = 5.0
    orbit_speed = 0.1  
    timestep = 0.01
    total_time = 20.0
    time = 0

    
    while time < total_time:
        
        angle = orbit_speed * time
        new_pos = chrono.ChVector3d(
            orbit_radius * math.cos(angle),
            orbit_radius * math.sin(angle),
            0.5
        )
        lidar.SetOffsetPose(chrono.ChFramed(new_pos, lidar_rot))

        
        manager.Update()

        
        buffer = lidar.GetMostRecentBuffer()
        if buffer.HasData():
            data = buffer.GetDepthData()
            print(f"Time {time:.2f}s - Points: {len(data)}")
            if len(data) > 0:
                print(f"First point: {data[0]}")

        
        system.DoStepDynamics(timestep)
        time += timestep

if __name__ == "__main__":
    main()