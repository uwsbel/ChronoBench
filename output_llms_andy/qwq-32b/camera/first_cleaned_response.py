import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import math
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh_body = chrono.ChBodyEasyStatic()
mesh = chrono.ChTriangleMesh()
if not mesh.LoadWavefrontMesh("my_mesh.obj"):
    print("Error loading mesh")
    exit()

shape = chrono.ChTriangleMeshShape()
shape.SetMesh(mesh)
shape.SetName("mesh_shape")
mesh_body.AddAsset(shape)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)


sensor_mgr = sensors.ChSensorManager()
sensor_mgr.SetSystem(system)


cam_width = 640
cam_height = 480
fov_deg = 60
fov = math.radians(fov_deg)


initial_pos = chrono.ChVectorD(2, 0, 0)
lookat = chrono.ChVectorD(0, 0, 0)
up = chrono.ChVectorD(0, 1, 0)
cam_frame = chrono.ChFrameD()
cam_frame.SetLookAt(initial_pos, lookat, up)


cam = sensors.ChCameraSensor()
cam.Initialize(sensor_mgr, cam_frame, cam_width, cam_height, fov)
gaussian_noise = sensors.ChNoiseGaussian(0.0, 0.05)
cam.AddNoiseFilter(gaussian_noise)
sensor_mgr.AddSensor(cam)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Camera Simulation')
vis.SetCameraPosition(chrono.ChVectorD(2, 0, 2))
vis.SetCameraRotation(chrono.Q_from_AngAxis(math.pi/4, chrono.ChVectorD(0, 1, 0)))
system.SetVisualSystem(vis)
vis.Initialize()
vis.AssetBindAll()
vis.AssetUpdateAll()


T_end = 10.0
angular_velocity = 0.1  
R = 2.0
height = 0.5

while system.GetChTime() < T_end:
    current_time = system.GetChTime()
    theta = angular_velocity * current_time
    
    
    x = R * math.cos(theta)
    y = R * math.sin(theta)
    new_pos = chrono.ChVectorD(x, y, height)
    cam_frame.SetLookAt(new_pos, lookat, up)
    cam.SetFrame_Csys(cam_frame)
    
    
    sensor_mgr.Update()
    image_data = cam.GetImageData()
    print(f"Time: {current_time:.2f}")
    print(f"Image shape: {image_data.shape}")
    print(f"First pixel RGB: {image_data[0,0]}")
    
    
    system.DoStepDynamics()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()