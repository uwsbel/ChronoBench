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






sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(body)


camera = chrono.ChCameraSensor()
camera.SetBody(body)
camera.SetResolution(640, 480)
camera.SetFOV(60)
camera.SetNearClip(0.1)
camera.SetFarClip(100)


noise_filter = chrono.ChNoiseFilterGaussian()
noise_filter.SetMean(0)
noise_filter.SetStdDev(0.01)
camera.AddFilter(noise_filter)


camera.SetVisualize(True)  


sensor_manager.AddCamera(camera)





orbit_radius = 2.0
orbit_speed = 0.1
time = 0.0




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(orbit_radius, orbit_radius, orbit_radius), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    time += 0.01
    x = orbit_radius * np.cos(time * orbit_speed)
    z = orbit_radius * np.sin(time * orbit_speed)
    camera_pos = chrono.ChVectorD(x, orbit_radius, z)
    vis.GetCamera().SetPosition(camera_pos)
    vis.GetCamera().LookAt(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))

    
    if camera.IsActive():
        buffer = camera.GetBuffer()
        if buffer is not None:
            
            print(f"Camera Buffer Data (first pixel): {buffer[0]}")

    system.DoStepDynamics(0.01)