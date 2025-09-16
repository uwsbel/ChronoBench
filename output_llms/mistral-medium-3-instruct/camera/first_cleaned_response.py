import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '../../data/'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mesh with Orbiting Camera')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 1.5, 5), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/obj/torus.obj'), False, True)


body = chrono.ChBody()
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetBodyFixed(True)  
body.GetVisualModel().AddTriangleMesh(mesh, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33<>(1), True)
system.Add(body)


camera = chrono.ChCameraSensor()
camera.SetName("orbiting_camera")
camera.SetCameraUp(chrono.ChVectorD(0, 0, 1))
camera.SetCameraLocation(chrono.ChVectorD(2, 0, 0))
camera.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
camera.SetFocalLength(0.035)  
camera.SetResolution(640, 480)
camera.SetFPS(30)
camera.SetAntiAliasing(chrono.ChCameraSensor::ANTIALIASING_4X)


camera.AddNoise(chrono.ChNoiseWhite(0.01))  
camera.AddNoise(chrono.ChNoiseGaussian(0.005))  


sensor_manager = chrono.ChSensorManager(system)
sensor_manager.AddSensor(camera)


vis.AddSensor(camera)
vis.EnableSensor(camera, True)


time_step = 0.01
end_time = 10.0
output_step = 0.1


current_time = 0
while vis.Run() and current_time < end_time:
    
    angle = current_time * 0.5  
    radius = 2.0
    camera_x = radius * np.cos(angle)
    camera_y = radius * np.sin(angle)
    camera.SetCameraLocation(chrono.ChVectorD(camera_x, camera_y, 0.5))

    
    camera.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if current_time % output_step < time_step:
        print(f"Time: {current_time:.2f}s - Camera position: ({camera_x:.2f}, {camera_y:.2f}, 0.5)")

    
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    sensor_manager.Update()