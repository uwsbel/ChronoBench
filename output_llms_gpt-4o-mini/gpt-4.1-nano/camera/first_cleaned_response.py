import pychrono as chrono
import pychrono.sensor as sensors
import math
import numpy as np


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




mesh_file = "path/to/your_mesh.obj"  



mesh_body = chrono.ChBodyEasyMesh(mesh_file,  
                                  100,        
                                  True,       
                                  True,       
                                  None,       
                                  sys=system)


mesh_body.SetBodyFixed(True)




system.Add(mesh_body)




sensor_manager = sensors.ChSensorManager()
sensor_manager.scene_manager.SetRenderMode(sensors.SceneManager.SENSOR_VISUALIZATION)



camera = sensors.ChCameraSensor(
    mesh_body,               
    20,                     
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChMatrix33D(1),     
    640,                     
    480,                     
    30                       
)


noise_filter = sensors.ChSensorNoiseFilter()
noise_filter.EnableGaussian(0.0)  

camera.AddFilter(noise_filter)


visualization_filter = sensors.ChFilterVisualizeCam()
camera.AddFilter(visualization_filter)


sensor_manager.AddSensor(camera)




total_time = 10.0  
time_step = 1.0 / 60.0  
num_steps = int(total_time / time_step)


orbit_radius = 2.0
orbit_speed = 0.5  
orbit_inclination = math.radians(30)  


def compute_camera_position(t):
    angle = orbit_speed * t
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle) * math.sin(orbit_inclination)
    z = orbit_radius * math.sin(angle) * math.cos(orbit_inclination)
    return chrono.ChVectorD(x, y, z)




for step in range(num_steps):
    current_time = step * time_step

    
    cam_pos = compute_camera_position(current_time)

    
    mesh_center = mesh_body.GetPos()
    direction = mesh_center - cam_pos
    direction.Normalize()
    
    
    camera.SetPos(cam_pos)
    camera.SetLookAt(mesh_center)

    
    sensor_manager.SetNextDrawWindow(0)  
    sensor_manager.Update()

    
    if camera.GetBuffer():
        image_buffer = camera.GetBuffer()
        
        print(f"Time {current_time:.2f}s: Received camera buffer with size {len(image_buffer)} bytes.")
        
    else:
        print(f"Time {current_time:.2f}s: No camera buffer available.")

    
    system.DoStepDynamics(time_step)




print("Simulation completed.")