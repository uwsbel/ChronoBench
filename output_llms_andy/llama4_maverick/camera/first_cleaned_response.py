import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + "models/bullet/robot.obj")
mesh.Transform(chrono.ChFrame(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))


body = chrono.ChBody()
body.SetBodyFixed(True)
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
body.AddVisualShape(mesh_shape)
sys.Add(body)


camera = sens.ChCameraSensor(
    body, 
    30,  
    chrono.ChFrame(chrono.ChVectorD(0, 0, 3), chrono.Q_ROTATE_Z_TO_Y),  
    640,  
    480,  
    chrono.ChFrad(chrono.CH_C_PI / 4)  
)


camera.PushFilter(sens.ChFilterRGBA8())
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterNoise(0.01, 0, 0.01, 0))  


manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


for i in range(1000):
    
    angle = i * chrono.CH_C_2PI / 360
    radius = 3
    x = radius * np.cos(angle)
    z = radius * np.sin(angle)
    camera.SetOffsetPose(chrono.ChFrame(chrono.ChVectorD(x, 0, z), chrono.Q_ROTATE_Z_TO_Y))
    
    
    sys.Update()
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print("Camera Buffer:", buffer.Buffer)

    
    sys.DoStepDynamics(0.01)