import pychrono as chrono
import pychrono.sensor as sens
import math


system = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('mesh.obj', True, True)  
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)


camera_body = chrono.ChBody()
camera_body.SetPos(chrono.ChVectorD(2, 0, 1))  
system.Add(camera_body)


manager = sens.ChSensorManager(system)


camera = sens.ChCameraSensor(
    camera_body,
    30,  
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    1280,  
    720,   
    chrono.CH_C_PI / 3  
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0)
manager.AddSensor(camera)


noise_filter = sens.ChFilterNoiseModelGaussian(camera, 0.1)
manager.AddFilter(noise_filter)


vis_filter = sens.ChFilterVisualize(camera, 1280, 720, "Camera Feed")
manager.AddFilter(vis_filter)


time_step = 0.01
end_time = 10.0
orbit_radius = 2.0
omega = 0.5  
height = 1.0


while system.GetChTime() < end_time:
    
    current_time = system.GetChTime()
    angle = omega * current_time
    new_x = orbit_radius * math.cos(angle)
    new_y = orbit_radius * math.sin(angle)
    new_z = height
    new_pos = chrono.ChVectorD(new_x, new_y, new_z)
    camera_body.SetPos(new_pos)
    
    
    dir_to_origin = chrono.ChVectorD(0, 0, 0) - new_pos
    dir_to_origin.Normalize()
    rot_quat = chrono.ChQuaternionD()
    rot_quat.SetFromTwoVectors(chrono.ChVectorD(0, 0, -1), dir_to_origin)
    camera_body.SetRot(rot_quat)
    
    
    system.DoStepDynamics(time_step)
    
    
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer:
        print(f"Time: {current_time:.2f}s, Buffer timestamp: {buffer.GetTimestamp():.2f}, Data available: {buffer.GetData() is not None}")