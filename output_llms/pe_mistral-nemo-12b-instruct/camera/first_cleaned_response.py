import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensors as sensors


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', 'data'))



my_system = chrono.ChSystemSMC()


mesh_path = chrono.GetChronoDataFile('triangle.obj')
my_mesh = chrono.ChTriangleMeshConnected()
my_mesh.LoadWavefrontMesh(mesh_path)


body_mesh = chrono.ChBody()
body_mesh.SetBodyFixed(True)
body_mesh.AddVisualShape(chrono.ChVisualShapeTriangleMesh(mesh_path, chrono.ChColor(0.5, 0.5, 0.5)))
my_system.Add(body_mesh)



camera_sensor = sensors.ChCameraSensor()
camera_sensor.SetWindowSize(640, 480)
camera_sensor.SetFocalLength(50)
camera_sensor.SetFocalPoint(chrono.ChVector3d(0, 0, 0))
camera_sensor.SetPrincipalPoint(chrono.ChVector2d(320, 240))
camera_sensor.SetUpVector(chrono.VECT_Z)
camera_sensor.SetViewVector(chrono.VECT_X)
camera_sensor.SetBody(body_mesh)
my_system.Add(camera_sensor)


sensor_manager = sensors.ChSensorManager()
sensor_manager.AddCameraSensor(camera_sensor)


camera_sensor.SetNoiseFilter(sensors.ChNoiseFilter())
camera_sensor.SetVisualization(sensors.ChVisualization())


camera_controller = chrono.ChCameraController()
camera_controller.SetBody(body_mesh)
camera_controller.SetCamera(camera_sensor)
camera_controller.SetOrbitDistance(5)
camera_controller.SetOrbitElevation(math.pi / 2)
camera_controller.SetOrbitAzimuth(0)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono Mesh Visualization')
vis.Initialize()


while vis.Run():
    
    camera_controller.DoStep()

    
    camera_buffer = camera_sensor.GetImageBuffer()
    print(f'Camera buffer data: {camera_buffer}')

    
    my_system.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()