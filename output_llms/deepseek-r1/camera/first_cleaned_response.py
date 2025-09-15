import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np


system = chrono.ChSystemNSC()


mesh_body = chrono.ChBodyEasyMesh(
    "mesh.obj",  
    1000,        
    True,        
    True,        
    False        
)
mesh_body.SetFixed(True)
system.Add(mesh_body)


camera_body = chrono.ChBodyAuxRef()
camera_body.SetBodyFixed(False)  
system.Add(camera_body)


orbit_radius = 5.0          
orbit_height = 1.5          
orbit_speed = 0.5           
sensor_update_rate = 30     
image_width, image_height = 1280, 720


sensor_manager = sens.ChSensorManager(system)


cam = sens.ChCameraSensor(
    camera_body,            
    sensor_update_rate,     
    chrono.ChFrameD(chrono.ChVectorD(0, orbit_height, 0), chrono.QUNIT),  
    image_width,            
    image_height,           
    math.radians(60)        
)
cam.SetName("Orbital Camera")
cam.SetLag(0.0)
cam.SetCollectionWindow(1/sensor_update_rate)


cam.AddNoiseModel(sens.ChNoiseNormal(0.0, 0.02))


cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera Feed"))


sensor_manager.AddSensor(cam)


time_step = 0.01
sim_duration = 10.0


while system.GetChTime() < sim_duration:
    current_time = system.GetChTime()
    
    
    angle = orbit_speed * current_time
    x = orbit_radius * math.cos(angle)
    z = orbit_radius * math.sin(angle)
    camera_body.SetPos(chrono.ChVectorD(x, orbit_height, z))
    
    
    forward = (mesh_body.GetPos() - camera_body.GetPos()).GetNormalized()
    up = chrono.ChVectorD(0, 1, 0)
    right = up.Cross(forward)
    rot_matrix = chrono.ChMatrix33D()
    rot_matrix.Set_A_axis(right, up, forward)
    camera_body.SetRot(rot_matrix.Get_A_quaternion())
    
    
    system.DoStepDynamics(time_step)
    
    
    sensor_manager.Update()
    
    
    buffer = cam.GetMostRecentBuffer()
    if buffer.HasData() and buffer.GetRGBA8Data() is not None:
        img_data = buffer.GetRGBA8Data()
        center_pixel = img_data[image_height//2, image_width//2]
        print(f"Time {current_time:.2f}s - Center pixel RGB: {center_pixel[:3]}")

print("Simulation completed successfully.")