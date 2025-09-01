import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/directory")
system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("path/to/your/mesh.obj", False, True)


mesh_body = chrono.ChBodyAuxRef()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)

visualization_mesh = chrono.ChTriangleMeshShape()
visualization_mesh.SetMesh(mesh)
mesh_body.AddAsset(visualization_mesh)

system.Add(mesh_body)


manager = sens.ChSensorManager(system)


cam_pos = chrono.ChVectorD(-5, 0, 0)
cam_target = chrono.ChVectorD(0, 0, 0)
cam_up = chrono.ChVectorD(0, 1, 0)

camera = sens.ChCameraSensor(
    mesh_body,                 
    30.0,                      
    chrono.ChFrameD(cam_pos),  
    1280,                      
    720,                       
    1.2                        
)


camera.SetName("Camera Sensor")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)


camera.PushFilter(sens.ChFilterCameraNoiseNormal(0.0, 0.1, 0.1))
camera.PushFilter(sens.ChFilterSave("camera/"))


manager.AddSensor(camera)


time_step = 0.01
sim_time = 0
orbit_radius = 5
orbit_speed = 0.5

while sim_time < 10:  
    
    angle = orbit_speed * sim_time
    new_cam_pos = chrono.ChVectorD(
        orbit_radius * math.cos(angle),
        0,
        orbit_radius * math.sin(angle)
    )
    camera.SetOffsetPose(chrono.ChFrameD(new_cam_pos, chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 1, 0))))

    
    system.DoStepDynamics(time_step)
    manager.Update()

    
    camera_data = camera.GetMostRecentRGBA8Buffer()
    if camera_data.HasData():
        rgba8_data = camera_data.GetRGBA8Data()
        print(f"Time: {sim_time:.2f}, Camera data shape: {rgba8_data.shape}")

    sim_time += time_step


manager.ResetCamera(camera)