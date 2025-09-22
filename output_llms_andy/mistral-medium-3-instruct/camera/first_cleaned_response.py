import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '../data/'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Orbiting Camera Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChVectorD(0, 0, 1))



mesh_filename = "path/to/your/mesh.obj"  
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_filename, False, True)


body = chrono.ChBody()
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetBodyFixed(True)
body.GetVisualModel().AddTriangleMesh(mesh)
system.Add(body)


sensor_manager = chrono.ChSensorManager(system)
sensor_manager.SetSensorsUpdateMode(chrono.SensorUpdateMode_ASYNCHRONOUS)


camera = chrono.ChCameraSensor(
    body,                          
    10,                            
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2)),  
    640,                           
    480,                           
    chrono.ChCameraSensor::LENS_PERSPECTIVE,
    1.414,                         
    0.05,                          
    chrono.ChCameraSensor::FILTER_BILINEAR
)


camera.SetNoiseFilter(chrono.ChCameraSensor::NoiseModel_GAUSSIAN, 0.05)  
camera.SetBlurFilter(chrono.ChCameraSensor::BlurFilter_GAUSSIAN, 3)  


sensor_manager.AddSensor(camera)


camera_vis = chrono.ChCameraSensorVisualization(camera)
camera_vis.SetWindowTitle("Camera View")
camera_vis.SetWindowSize(640, 480)
camera_vis.Initialize()


time_step = 0.01
sim_time = 10.0
orbit_radius = 2.0
orbit_speed = 0.5


current_time = 0
while vis.Run() and current_time < sim_time:
    
    angle = orbit_speed * current_time
    camera_pos = chrono.ChVectorD(
        orbit_radius * np.cos(angle),
        orbit_radius * np.sin(angle),
        1.0
    )
    camera.SetCameraTransform(chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 0, 1))))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update()

    
    if camera.HasData():
        print(f"Time: {current_time:.2f}s - Camera buffer size: {camera.GetBufferSize()}")

    
    system.DoStepDynamics(time_step)
    current_time += time_step


camera_vis.Shutdown()
vis.Shutdown()