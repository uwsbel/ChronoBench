import pychrono as chrono
import pychrono.sensor
import pychrono.irrlicht
import math






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
camera.Setup(body, chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
camera.SetResolution(640, 480)
camera.SetFOV(math.pi / 3.0)
camera.SetNearClip(0.1)
camera.SetFarClip(100)


noise_filter = chrono.ChNoiseFilter()
noise_filter.SetNoiseType(chrono.ChNoiseFilter.NOISE_GAUSSIAN)
noise_filter.SetNoiseLevel(0.01)  
camera.AddFilter(noise_filter)


camera.SetVisualize(True)
camera.SetBackground(chrono.ChColor(0.2, 0.2, 0.2))

sensor_manager.AddCamera(camera)


time_step = 0.01
simulation_time = 10


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(camera.GetCamera())
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    if camera.IsActive():
        buffer = camera.GetBuffer()
        if buffer:
            print(f"Camera buffer data size: {len(buffer)}")
            
            
    
    
    time = system.GetChTime()
    orbit_radius = 2.0
    camera_position = chrono.ChVectorD(
        orbit_radius * math.cos(time),
        0.5,
        orbit_radius * math.sin(time)
    )
    camera.SetPosition(camera_position)
    camera.SetAimPoint(chrono.ChVectorD(0, 0, 0))