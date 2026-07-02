import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


mesh_body = chrono.ChBodyEasyMesh("mesh.obj", 1000, True, True, chrono.ChVisualShapeMesh.Show_Faces)
mesh_body.SetName("Fixed Mesh Body")
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
system.Add(mesh_body)


lidar_body = chrono.ChBody()
lidar_body.SetName("Lidar Carrier")
lidar_body.SetPos(chrono.ChVector3d(5, 0, 1))  
lidar_body.SetFixed(True)  
system.Add(lidar_body)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 10), 1, 1, 1)
manager.scene.SetAmbientLight(chrono.ChColor(0.4, 0.4, 0.4))


lidar = sens.ChLidarSensor(
    lidar_body,  
    10,          
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)),  
    360,         
    1,           
    -chrono.CH_PI / 2,  
    chrono.CH_PI / 2,   
    100          
)
lidar.SetName("Orbiting Lidar")
lidar.SetLidarRender(sens.ChVisualSystem.Type_LidarPoints)
lidar.SetSave(True)
lidar.SetFilename("lidar_data_{}.csv")


lidar.AddNoiseModel(sens.ChLidarNoiseXYZNormal(0.01, 0.01, 0.01))

manager.AddSensor(lidar)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Orbiting Lidar Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 8), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()


time_step = 1e-3
orbit_radius = 5
orbit_speed = 0.5  
simulation_time = 5.0


while system.GetChTime() < simulation_time:
    
    angle = orbit_speed * system.GetChTime()
    lidar_body.SetPos(chrono.ChVector3d(
        orbit_radius * math.cos(angle),
        orbit_radius * math.sin(angle),
        1
    ))
    
    
    manager.Update()
    
    
    system.DoStepDynamics(time_step)
    
    
    if lidar.GetMostRecentBuffer():
        buffer = lidar.GetMostRecentBuffer().GetMetadata()
        print(f"Time {system.GetChTime():.4f}: {buffer.shape[0]} lidar points")
        if buffer.shape[0] > 0:
            print("Sample points:", buffer[:3, :3])

    vis.Render()