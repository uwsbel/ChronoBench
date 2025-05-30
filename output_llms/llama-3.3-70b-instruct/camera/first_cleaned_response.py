import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystem()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("PyChrono Simulation")


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh("mesh.obj")


body = chrono.ChBodyEasyMesh(sys, mesh, True, False)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
sys.Add(body)


cam = sens.ChCameraSensor(sys)


sens_man = sens.ChSensorManager(sys)
sens_man.AddSensor(cam)


cam.AddFilter(sens.ChGaussianNoiseFilter(0.1))
cam.AddFilter(sens.ChSaltAndPepperNoiseFilter(0.05))
cam.AddVisualization(sens.ChImageDisplay())


dt = 0.01
t_end = 10.0


t = 0.0


cam_buffer_data = []

while t < t_end:
    
    radius = 2.0
    angle = t * 0.1
    cam_pos = chrono.ChVectorD(radius * np.cos(angle), radius * np.sin(angle), 1.0)
    cam.SetPos(cam_pos)

    
    sys.DoStepDynamics(dt)

    
    buffer_data = cam.GetBuffer()
    cam_buffer_data.append(buffer_data)

    
    print(f"Time: {t:.2f}, Camera Buffer Data: {buffer_data}")

    
    t += dt


vis.Run()