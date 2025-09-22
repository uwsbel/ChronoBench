import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import os
import math


print("Creating the Chrono system...")
chrono_system = chrono.ChSystemNSC()


chrono_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
chrono_system.Add(ground)


vis_material = chrono.ChVisualMaterial()
vis_material.SetAmbientColor(chrono.ChColor(0.2, 0.2, 0.2))
vis_material.SetDiffuseColor(chrono.ChColor(0.8, 0.3, 0.3))
vis_material.SetSpecularColor(chrono.ChColor(0.8, 0.8, 0.8))
vis_material.SetFresnelMin(0.2)
vis_material.SetFresnelMax(0.7)
vis_material.SetRoughness(0.1)


mesh_body = chrono.ChBodyEasyMesh(
    
    filename="mesh.obj",
    density=1000,
    
    visualize=True,
    
    collide=False
)
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.GetVisualShape(0).SetMaterial(0, vis_material)
chrono_system.Add(mesh_body)


manager = sens.ChSensorManager(chrono_system)
manager.scene.AddPointLight(chrono.ChVectorD(100, 100, 100), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))
manager.scene.SetSceneEpsilon(1e-3)


camera_update_rate = 30.0  
camera_lag = 0.0           
camera_collection_time = 0.0  


camera_pos = chrono.ChVectorD(5, 1, 0)
camera_target = chrono.ChVectorD(0, 0, 0)
camera_up = chrono.ChVectorD(0, 1, 0)


camera = sens.ChCameraSensor(
    mesh_body,                   
    camera_update_rate,          
    chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))), 
    1280,                        
    720,                         
    1.0                          
)


camera.SetName("Camera Sensor")
camera.SetLag(camera_lag)
camera.SetCollectionWindow(camera_collection_time)


camera.PushFilter(sens.ChFilterCameraSetPose(
    camera_target,  
    camera_up       
))


camera.PushFilter(sens.ChFilterCameraNoiseNormal(0.0, 0.02))


camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))





manager.AddSensor(camera)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono_system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Mesh with Camera Sensor Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 1, -6), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


step_size = 1e-3
sim_time = 0.0
orbit_radius = 5.0
orbit_height = 1.0
orbit_speed = 0.2  


timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    
    orbit_angle = sim_time * orbit_speed
    new_camera_pos = chrono.ChVectorD(
        orbit_radius * math.cos(orbit_angle),
        orbit_height,
        orbit_radius * math.sin(orbit_angle)
    )
    
    
    camera.SetOffsetPose(chrono.ChFrameD(
        new_camera_pos,
        chrono.Q_from_AngAxis(orbit_angle + math.pi/2, chrono.ChVectorD(0, 1, 0))
    ))
    
    
    camera.PushFilter(sens.ChFilterCameraSetPose(
        chrono.ChVectorD(0, 0, 0),  
        chrono.ChVectorD(0, 1, 0)   
    ))
    
    
    vis.BeginScene()
    vis.Render()
    manager.Update()
    chrono_system.DoStepDynamics(step_size)
    vis.EndScene()
    
    
    sim_time += step_size
    
    
    if int(sim_time * 10) % 10 == 0:
        
        camera_data = camera.GetMostRecentRGBA8Buffer()
        if camera_data.HasData():
            print(f"Time: {sim_time:.2f}s, Camera capturing data: {camera_data.Width}x{camera_data.Height}")
        
    
    timer.Spin(step_size)
    
    
    if sim_time > 20:
        break

print("Simulation complete!")