import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh_file = "cube.obj"  
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadFromFile(mesh_file)


body = chrono.ChBodyEasy()
body.AddAsset(mesh)
body.SetBodyFixed(True)  
system.Add(body)


body.SetPos(chrono.ChVectorD(0, 0, 0))






sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(body)


camera = chrono.ChCameraSensor()
camera.SetBody(body)
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetNearClip(0.1)
camera.SetFarClip(100)
camera.SetNoise(0.01, 0.01)  
sensor_manager.AddSensor(camera)


camera_vis = chronoirr.ChVisualisationCamera()
camera_vis.SetSensor(camera)
camera_vis.SetRenderMode(chronoirr.ChVisualisationCamera.RENDER_MODE_DEPTH)  
sensor_manager.AddVisualizer(camera_vis)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()







time_step = 0.01


simulation_duration = 10


orbit_radius = 2.0
orbit_speed = 0.5

current_time = 0.0
while vis.Run() and current_time < simulation_duration:
    
    angle = orbit_speed * current_time
    camera_x = orbit_radius * np.cos(angle)
    camera_z = orbit_radius * np.sin(angle)
    vis.GetCamera().SetPos(chrono.ChVectorD(camera_x, 1.5, camera_z))
    vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))

    
    system.DoStepDynamics(time_step)

    
    if camera.IsDataAvailable():
        buffer_data = camera.GetBufferData()
        print(f"Camera buffer data at time {current_time:.2f}: {buffer_data}")

    current_time += time_step