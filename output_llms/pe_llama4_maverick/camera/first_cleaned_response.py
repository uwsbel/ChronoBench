import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChronoDataPath('./data/')


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + 'robot.obj')


body_mesh = chrono.ChBody()
body_mesh.SetBodyFixed(True)  
body_mesh_shape = chrono.ChVisualShape()
body_mesh_shape.SetMesh(mesh)
body_mesh.AddVisualShape(body_mesh_shape)
sys.Add(body_mesh)


camera = sens.ChCameraSensor(
    body_mesh,  
    10,  
    chrono.ChFramed(chrono.ChVector3d(-5, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI / 2, chrono.ChVector3d(0, 1, 0))),  
    640,  
    480,  
    chrono.CH_PI / 3  
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Before Grayscale Filter"))
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Output"))
camera.PushFilter(sens.ChFilterSave(chrono.GetChronoDataPath() + "output/"))


manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


time_step = 1e-3
time_end = 10
orbit_radius = 5
orbit_omega = 0.1  
for time in np.arange(0, time_end, time_step):
    
    angle = orbit_omega * time
    x = orbit_radius * math.cos(angle)
    z = orbit_radius * math.sin(angle)
    camera.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(x, 0, z), chrono.Q_from_AngAxis(chrono.CH_PI / 2 + angle, chrono.ChVector3d(0, 1, 0))))
    
    
    sys.DoStepDynamics(time_step)
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print("Camera Buffer Data:")
        print(buffer.GetRGBA8().shape)
        


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)