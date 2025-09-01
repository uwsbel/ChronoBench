import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")  


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj")


mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetMutable(False)


body = chrono.ChBody()
body.SetBodyFixed(True)  
body.AddVisualShape(mesh_asset)


sys.Add(body)


camera = sens.ChCameraSensor(
    body,  
    30,    
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_X)),  
    640,   
    480,   
    chrono.CH_C_PI / 3  
)
camera.SetName("Camera Sensor")


noise_model = sens.ChNoiseCamera(
    sens.ChNoiseCameraModel(sens.ChNoiseCameraModel_CAMERA_NOISE_CONSTANT),
    0.01,  
    0.0,   
    0.0    
)
camera.PushFilter(noise_model)


camera.PushFilter(sens.ChFilterGrayscale())


vis = sens.ChVisualSensorInstance(camera)
vis.SetName("Camera Visualization")


manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


time_step = 1e-3
total_time = 10
time = 0


orbit_radius = 3
angle = 0
angle_step = 0.01

while time < total_time:
    
    angle += angle_step
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)
    camera.SetOffsetPose(chrono.ChFrameD(chrono.ChVector3d(x, 0, z), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_X)))
    
    
    sys.DoStepDynamics(time_step)
    
    
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print("Camera Buffer Data:")
        print("  Buffer: ", buffer)
        
    
    
    time += time_step

import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj")
mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetMutable(False)
body = chrono.ChBody()
body.SetBodyFixed(True)
body.AddVisualShape(mesh_asset)
sys.Add(body)


camera = sens.ChCameraSensor(
    body,
    30,
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 3), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_X)),
    640,
    480,
    chrono.CH_C_PI / 3
)
camera.SetName("Camera Sensor")
noise_model = sens.ChNoiseCamera(sens.ChNoiseCameraModel(sens.ChNoiseCameraModel_CAMERA_NOISE_CONSTANT), 0.01, 0.0, 0.0)
camera.PushFilter(noise_model)
camera.PushFilter(sens.ChFilterGrayscale())
vis = sens.ChVisualSensorInstance(camera)
vis.SetName("Camera Visualization")
manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)


time_step = 1e-3
total_time = 10
time = 0
orbit_radius = 3
angle = 0
angle_step = 0.01

while time < total_time:
    angle += angle_step
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)
    camera.SetOffsetPose(chrono.ChFrameD(chrono.ChVector3d(x, 0, z), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_X)))
    sys.DoStepDynamics(time_step)
    manager.Update()
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print("Camera Buffer Data:")
        print("  Buffer: ", buffer)
    time += time_step