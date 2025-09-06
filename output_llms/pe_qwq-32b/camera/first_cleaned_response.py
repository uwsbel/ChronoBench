import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as chsensor
import math
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mesh Camera Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChVectorD(0, 3, 10))  
vis.AddTypicalLights()


mesh_file = 'my_mesh.obj'  
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetYoungModulus(2e7)
material.SetRestitution(0.1)

mesh_body = chrono.ChBodyEasyMesh(
    chrono.GetChronoDataFile(mesh_file),
    1000,  
    True,  
    False,  
    material
)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetFixed(True)
system.Add(mesh_body)


sensor_mgr = chsensor.ChSensorManager(system)
cam_sensor = chsensor.ChCameraSensor()
cam_sensor.SetName("orbit_cam")
cam_sensor.SetResolution(800, 600)
cam_sensor.SetFov(60.0)  
cam_sensor.SetPixelFormat(chsensor.PixelType_RGB)  
cam_sensor.SetPosition(chrono.ChVectorD(0, 0, 5))  
cam_sensor.SetRotation(chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0)))  


cam_sensor.AddNoiseFilter(chsensor.ChNoiseFilterGaussian(0.0, 0.1))

sensor_mgr.AddSensor(cam_sensor)
sensor_mgr.SetVisualizationType(chsensor.VisualizationType_RGB)  


time_step = 0.01
angular_velocity = 0.1  
radius = 5.0  


while vis.Run():
    current_time = system.GetChTime()
    angle = angular_velocity * current_time
    
    
    x = radius * math.cos(angle)
    z = radius * math.sin(angle)
    cam_pos = chrono.ChVectorD(x, 0, z)
    cam_sensor.SetPosition(cam_pos)
    
    
    look_dir = chrono.ChVectorD(0, 0, 0) - cam_pos
    up_dir = chrono.ChVectorD(0, 1, 0)
    cam_sensor.SetRotation(chrono.Q_from_dir_dir(up_dir, look_dir))
    
    
    sensor_mgr.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    
    buffer = cam_sensor.GetSensor().GetColorBuffer()
    if buffer is not None:
        print(f"Frame {int(current_time/time_step)}:")
        print(f"  Resolution: {buffer.shape}")
        print(f"  Min pixel value: {buffer.min()}, Max pixel value: {buffer.max()}")


vis.Close()