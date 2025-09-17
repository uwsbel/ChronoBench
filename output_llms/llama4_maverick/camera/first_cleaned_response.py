import pychrono as chrono
import pychrono.sensor as sens
import numpy as np


chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  



mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/forklift/meshes/forklift_body.obj"))
mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))  


body = chrono.ChBody()
body.SetBodyFixed(True)  
body_shape = chrono.ChTriangleMeshShape()
body_shape.SetMesh(mesh)
body.AddVisualShape(body_shape)
chrono_system.Add(body)



body.SetPos(chrono.ChVectorD(0, 0, 0))


offset_pose = chrono.ChFrameD(chrono.ChVectorD(-5, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
camera = sens.ChCameraSensor(
    body, 
    30,  
    offset_pose,
    640,  
    480,  
    chrono.ChFrad(chrono.CH_C_PI / 4)  
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8())
camera.PushFilter(sens.ChFilterGrayscale())
camera.PushFilter(sens.ChFilterNoiseSaltPepper(0.1))  
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Output"))


manager = sens.ChSensorManager(chrono_system)
manager.AddSensor(camera)


time_step = 1e-3
time_end = 10
for time in np.arange(0, time_end, time_step):
    
    orbit_radius = 5
    camera_angle = time * 0.1  
    camera_pos = chrono.ChVectorD(orbit_radius * np.cos(camera_angle), 0, orbit_radius * np.sin(camera_angle))
    offset_pose = chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(camera_angle, chrono.ChVectorD(0, 1, 0)))
    camera.SetOffsetPose(offset_pose)
    
    
    chrono_system.Update()
    manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Time: {time}, Camera Buffer Data (grayscale): {buffer.GetRGBA8().shape}")

    
    chrono_system.DoStepDynamics(time_step)


if __name__ == "__main__":
    pass