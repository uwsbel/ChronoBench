import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


mesh_file = "cube.obj"  
mesh_body = chrono.ChBodyEasy()
mesh_body.SetBodyFixed(True)
mesh_body.AddAsset(chrono.ChTriangleMeshConnected())
mesh_body.GetAsset().LoadFromFile(mesh_file)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)


camera_sensor = chrono.ChSensorCamera()
camera_sensor.Setup(mesh_body, chrono.ChVectorD(0, 0, -1),  
                    640, 480,  
                    0.1, 100,  
                    60)  


noise_filter = chrono.ChSensorNoiseGaussian()
noise_filter.SetNoiseLevel(0.01)  
camera_sensor.AddFilter(noise_filter)


sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(camera_sensor)
system.Add(sensor_manager)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))  
vis.AddTypicalLights()


orbit_radius = 3.0
orbit_speed = 0.1
angle = 0.0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    angle += orbit_speed
    camera_pos = chrono.ChVectorD(orbit_radius * np.cos(angle), 2, orbit_radius * np.sin(angle))
    vis.GetIrrlichtApplication().GetSceneManager().GetActiveCamera().SetPosition(camera_pos)
    vis.GetIrrlichtApplication().GetSceneManager().GetActiveCamera().LookAt(chrono.ChVectorD(0,0,0))

    
    if sensor_manager.GetSensorCount() > 0:
        camera_data = sensor_manager.GetSensorData(0)
        if camera_data is not None:
            
            print(f"Timestamp: {camera_data.timestamp}")
            print(f"Image width: {camera_data.width}")
            print(f"Image height: {camera_data.height}")

            
            image_data = camera_data.image
            
            
            
            
    
    system.DoStepDynamics(0.01)