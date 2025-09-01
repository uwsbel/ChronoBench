import os
import math
import numpy as np
import matplotlib.pyplot as plt
import pychrono as chrono
import pychrono.sensor as sens
import pychrono irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)
sys.SetSolverMaxIterations(100)
sys.SetSolverTolerance(1e-6)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowTitle("Mesh Visualization with Camera")
vis.SetWindowSize(1024, 768)
vis.SetNumberOfCameras(1)
vis.AttachSystem(sys)
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()


mesh_path = chrono.GetChronoDataFile("meshes/teapot.obj")
mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(mesh_path, False)


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetFixed(True)


mesh_shape = chrono.ChVisualShapeMesh()
mesh_shape.SetMesh(mesh)
body.AddVisualShape(mesh_shape)
sys.Add(body)


sensor_manager = sens.ChSensorManager(sys)


camera = sens.ChCameraSensor()
camera.SetDimensions(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(5, 0, 0))
camera.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))


noise_model = sens.ChNoiseGaussian()
noise_model.SetMean(0.0)
noise_model.SetStdDev(0.01)
camera.AddNoiseModel(noise_model)


sensor_manager.AddSensor(camera)


fig, ax = plt.subplots()
plt.ion()
plt.show()


total_time = 10.0
step_size = 0.01
num_steps = int(total_time / step_size)
angle = 0.0


for step in range(num_steps):
    
    radius = 5.0
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    camera.SetPosition(chrono.ChVector3d(x, y, 0))
    camera.SetRotation(chrono.ChQuaternionD(chrono.Q_from_AngY(angle)))
    angle += 0.02
    
    
    sensor_manager.Update()
    image = camera.GetImageData()
    
    
    img_array = np.array(image.GetData(), dtype=np.uint8).reshape(image.GetHeight(), image.GetWidth(), 4)
    ax.imshow(img_array)
    plt.pause(0.001)
    
    
    print(f"Step {step}: Camera buffer data (first 10 elements): {image.GetData()[0:10]}")
    
    
    sys.DoStepDynamics(step_size)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Close()