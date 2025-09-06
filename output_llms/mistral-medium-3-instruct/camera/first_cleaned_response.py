import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '..', 'data'))
chrono.SetChronoOutputPath(os.path.join(os.path.dirname(__file__), '..', 'output'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Triangular Mesh with Orbiting Camera')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.SetCameraMoveScale(0.05)


mesh_path = "path/to/your/mesh.obj"  
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, False, True)


fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
fixed_body.GetVisualModel().AddTriangleMesh(mesh)
fixed_body.GetCollisionModel().AddTriangleMesh(mesh)
system.Add(fixed_body)


camera = sensors.ChCameraSensor(
    fixed_body,                
    10,                        
    chrono.ChFrameD(chrono.ChVectorD(5, 0, 0), chrono.QUNIT),  
    640,                       
    480,                       
    1.0                        
)


camera.SetLensModel(sensors.ChLensModelPINHOLE)
camera.SetImagePlaneSize(640, 480)
camera.SetClippingPlanes(0.1, 100)
camera.SetUseColorTexture(True)


noise_filter = sensors.ChFilterGaussianNoise()
noise_filter.SetNoiseLevel(0.01)  
camera.AddFilter(noise_filter)


visualization_filter = sensors.ChFilterVisualize(640, 480, "Camera View")
camera.AddFilter(visualization_filter)


sensor_manager = sensors.ChSensorManager(system)
sensor_manager.AddSensor(camera)


time_step = 0.01
end_time = 10.0


while vis.Run():
    
    time = system.GetChTime()
    radius = 5.0
    angle = time * 0.5  

    
    cam_pos = chrono.ChVectorD(
        radius * np.cos(angle),
        radius * np.sin(angle),
        2.0  
    )

    
    camera.SetCameraToBodyOffset(chrono.ChFrameD(cam_pos, chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.ChVectorD(0,0,1))))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if camera.GetLatestFrame() is not None:
        frame = camera.GetLatestFrame()
        print(f"Time: {time:.2f}s | Camera position: {cam_pos} | Buffer size: {frame.GetBufferWidth()}x{frame.GetBufferHeight()}")

    
    system.DoStepDynamics(time_step)

    
    if time >= end_time:
        break


sensor_manager.RemoveSensor(camera)