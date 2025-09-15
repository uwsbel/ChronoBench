import os
import math
import chrono.chrono as chrono
import chrono.chrono_irrlicht as chronoirr
import chrono.chrono_sensor as sens

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  

    
    mesh_body = chrono.ChBodyEasyMesh(
        os.path.join(chrono.GetChronoDataPath(), "models", "cube.obj"),  
        1000,  
        True,  
        True   
    )
    mesh_body.SetFixed(True)
    system.Add(mesh_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 0), chrono.ChColor(1, 1, 1), 500)

    
    orbit_radius = 5.0
    orbit_speed = 0.5  
    lidar_params = sens.LidarModelParams(
        horz_res=360,    
        vert_res=180,    
        hfov=2*math.pi,  
        vfov=math.pi,    
        max_distance=100,
        lag=0.0,
        angle_lag=0.0
    )
    noise_model = sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.05, 0.05, 0.05))

    
    parent_body = chrono.ChBody()
    parent_body.SetBodyFixed(False)
    parent_body.SetPos(chrono.ChVectorD(orbit_radius, 0, 0))
    system.Add(parent_body)

    
    lidar = sens.ChLidarSensor(
        parent_body,             
        30,                      
        chrono.ChFrameD(),       
        lidar_params             
    )
    lidar.SetName("Orbiting Lidar")
    lidar.SetNoiseModel(noise_model)
    lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Data"))
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))
    manager.AddSensor(lidar)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Lidar Orbit Simulation')
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(0, 3, -5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    time_step = 0.01
    time = 0.0
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        theta = orbit_speed * time
        new_pos = chrono.ChVectorD(
            orbit_radius * math.cos(theta),
            0,
            orbit_radius * math.sin(theta)
        )
        parent_body.SetPos(new_pos)
        parent_body.SetRot(chrono.Q_from_AngAxis(theta + math.pi/2, chrono.ChVectorD(0, 1, 0)))

        
        manager.Update()

        
        buffer = lidar.GetMostRecentBuffer()
        if buffer and buffer.HasData():
            print(f"Time: {time:.2f} | Points captured: {buffer.GetXYZ().shape[0]}")
            if buffer.GetXYZ().shape[0] > 0:
                print(f"Sample point: {buffer.GetXYZ()[0]}")

        
        system.DoStepDynamics(time_step)
        time += time_step

if __name__ == "__main__":
    main()