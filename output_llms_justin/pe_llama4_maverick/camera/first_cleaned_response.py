import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data')


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/tractor.obj'))


body_mesh = chrono.ChBody()
body_mesh.SetBodyFixed(True)  
body_mesh_shape = chrono.ChVisualShapeTriangleMesh()
body_mesh_shape.SetMesh(mesh)
body_mesh.AddVisualShape(body_mesh_shape)
sys.Add(body_mesh)


body_camera = chrono.ChBody()
body_camera.SetPos(chrono.ChVector3d(0, 2, 0))  
sys.Add(body_camera)


camera = sens.ChCameraSensor(
    body_camera,  
    10,  
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI, chrono.ChVector3d(0, 1, 0))),  
    640,  
    480,  
    chrono.CH_PI / 4  
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8RGBA8())  
camera.PushFilter(sens.ChFilterGrayscale())  
camera.PushFilter(sens.ChFilterNoisePixDep(0.03, 0.001, 123))  
camera.PushFilter(sens.ChFilterVisualize(640, 480, 1))  


manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


time_step = 0.02  
t = 0  
while t < 10:  
    
    angle = t * 0.1  
    radius = 2
    x = radius * np.cos(angle)
    z = radius * np.sin(angle)
    body_camera.SetPos(chrono.ChVector3d(x, 2, z))
    
    
    sys.DoStepDynamics(time_step)
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Camera Buffer at time {t}: {buffer.GetRGBA8().shape}")
    
    
    t += time_step

print("Simulation completed.")