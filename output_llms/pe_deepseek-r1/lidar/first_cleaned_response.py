import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np
import os




system = chrono.ChSystemNSC()
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  





mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)  
system.Add(mesh_body)


mesh_path = chrono.GetChronoDataFile("models/cube.obj")  
mesh_visual = chrono.ChVisualShapeModelFile()
mesh_visual.SetFilename(mesh_path)
mesh_body.AddVisualShape(mesh_visual)




manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 100)


lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0))  
lidar = sens.ChLidarSensor(
    mesh_body,          
    30,                 
    lidar_offset,       
    1200,               
    720,                
    chrono.CH_PI / 3,   
    chrono.CH_PI / 4    
)


lidar.SetName("Orbiting Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.02)


noise_model = sens.ChNoiseNormalDist(0.0, 0.01)  
lidar.AddNoiseModel(noise_model)


vis = sens.ChVisualSystemSensor()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Point Cloud")
lidar.PushFilter(sens.ChFilterVisualize(1280, 720, "Raw Lidar Data"))


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))

manager.AddSensor(lidar)




vis_sys = chronoirr.ChVisualSystemIrrlicht()
vis_sys.AttachSystem(system)
vis_sys.SetWindowSize(1024, 768)
vis_sys.SetWindowTitle("Orbiting Lidar Simulation")
vis_sys.Initialize()
vis_sys.AddLogo()
vis_sys.AddSkyBox()
vis_sys.AddCamera(chrono.ChVector3d(3, 3, 3))
vis_sys.AddTypicalLights()




orbit_radius = 3.0
orbit_speed = 0.5  
time_step = 0.01
current_time = 0.0

while vis_sys.Run():
    
    x = orbit_radius * np.cos(orbit_speed * current_time)
    z = orbit_radius * np.sin(orbit_speed * current_time)
    lidar_offset.SetPos(chrono.ChVector3d(x, 1.5, z))
    lidar.SetOffsetPose(lidar_offset)
    
    
    manager.Update()
    
    
    buffer = lidar.GetMostRecentBuffer()
    if buffer.HasData():
        data = buffer.GetXYZIData()
        print(f"Time: {current_time:.2f}s | Points: {data.width * data.height} | "
              f"Position: ({x:.2f}, 1.50, {z:.2f})")
    
    
    vis_sys.BeginScene()
    vis_sys.Render()
    vis_sys.RenderFrame(chrono.ChFramed(lidar.GetOffsetPose()), 0.5)  
    vis_sys.EndScene()
    system.DoStepDynamics(time_step)
    current_time += time_step