import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data/')  


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + 'models/bullet/robot.obj')  


body = chrono.ChBody()
body_mesh = chrono.ChTriangleMeshShape()
body_mesh.SetMesh(mesh)
body.AddVisualShape(body_mesh)
body.SetBodyFixed(True)  
sys.Add(body)


camera = sens.ChCameraSensor(
    body, 
    30,  
    chrono.ChFrame(chrono.ChVector3f(0, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_Y)), 
    640,  
    480,  
    chrono.CH_C_PI / 3  
)


camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8())
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterNoise(0.1, 0.1, 0.1, 0.1))  
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Output"))


manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


time_step = 1e-3
time_end = 10
for time in np.arange(0, time_end, time_step):
    
    angle = time * 0.1
    radius = 3
    x = radius * np.cos(angle)
    z = radius * np.sin(angle)
    camera.SetOffset(chrono.ChVector3f(x, 0, z))
    camera.SetRot(chrono.Q_from_AngAxis(-angle, chrono.VECT_Y))
    
    
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Time: {buffer.Timestamp}, Buffer Size: {buffer.Size}, Data: {buffer.GetRGBA8().shape}")

    
    sys.DoStepDynamics(time_step)