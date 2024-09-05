import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import numpy as np


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


scene = chrono.ChScene()
scene.SetAmbientLight(chrono.ChVectorF(0.5, 0.5, 0.5))
scene.SetCamera(chrono.ChVectorF(0, 0, 0), chrono.ChVectorF(0, 0, -1))
sensor_manager = sens.ChSensorManager(scene)


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh('mesh.obj')
mesh.SetStatic(True)


body = chrono.ChBodyEasyMesh(mesh, 1000, True, True)
body.SetPos(chrono.ChVectorF(0, 0, 0))
body.SetRot(chrono.ChQuaternionF(1, 0, 0, 0))
system.Add(body)


camera = sens.ChCameraSensor()
camera.SetName('camera')
camera.SetPosition(chrono.ChVectorF(0, 0, 5))
camera.SetLookAt(chrono.ChVectorF(0, 0, 0))
camera.SetFov(60)
camera.SetResolution(640, 480)
camera.SetNearClippingPlane(0.1)
camera.SetFarClippingPlane(100)
camera.SetNoiseFilter(sens.ChNoiseFilter.GAUSSIAN, 0.1)
camera.SetVisualization(sens.ChVisualizationType.POINT_CLOUD)
sensor_manager.AddSensor(camera)


noise_filter = sens.ChNoiseFilter()
noise_filter.SetType(sens.ChNoiseFilter.GAUSSIAN)
noise_filter.SetIntensity(0.1)
camera.AddNoiseFilter(noise_filter)


visualization = sens.ChVisualization()
visualization.SetType(sens.ChVisualizationType.POINT_CLOUD)
camera.AddVisualization(visualization)


step_size = 0.01
time_end = 10
while system.GetChTime() < time_end:
    system.DoStepDynamics(step_size)
    camera.SetPosition(chrono.ChVectorF(5 * np.cos(system.GetChTime()), 0, 5 * np.sin(system.GetChTime())))
    camera.SetLookAt(chrono.ChVectorF(0, 0, 0))
    sensor_manager.Update()
    print(camera.GetBuffer().GetDepthData())


application = irr.ChIrrApp(scene, 'PyChrono Simulation', 800, 600)
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorF(0, 0, 0), chrono.ChVectorF(0, 0, -1))
application.AddCameraSensor(camera)
application.Run()