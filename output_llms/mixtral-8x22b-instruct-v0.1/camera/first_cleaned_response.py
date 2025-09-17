import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as chronopost
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'chrono_data'))
chrono.SetChronoVisualization(chronoirr.ChVisualization())
chrono.SetChronoDataPath(os.path.dirname(__file__))


my_system = chrono.ChSystemNSC()


mesh_file = "path/to/your/mesh.obj"
body = chrono.ChBodyEasyBox(1, 1, 1, 1000)
body.SetBodyFixed(True)
body.AddAsset(chrono.ChTriangleMeshShape(chrono.ChFileLoader.LoadTriangleMesh(mesh_file)))
my_system.AddBody(body)


camera_sensor = chrono.ChCameraSensor(chronoirr.ChIrrCamera(chronoirr.ChIrrApp().GetSystem(), 800, 600))
camera_sensor.SetName("Camera")
camera_sensor.SetDescription("Camera sensor")
camera_sensor.SetSensorOffset(chrono.ChVectorD(0, 0, 2))
camera_sensor.SetSensorDirection(chrono.ChVectorD(0, 0, -1))
camera_sensor.SetSensorUp(chrono.ChVectorD(0, 1, 0))
camera_sensor.SetSensorFarPlane(100)


sensor_manager = chrono.ChSensorManager(my_system)
sensor_manager.AddSensor(camera_sensor)


noise_filter = chrono.ChNoiseFilter()
noise_filter.SetNoiseType(chrono.ChNoiseFilter.FILTER_GAUSSIAN)
noise_filter.SetSigma(0.1)
camera_sensor.AddFilter(noise_filter)

image_visualization = chrono.ChImageVisualization()
image_visualization.SetName("Image visualization")
image_visualization.SetDescription("Image visualization")
camera_sensor.AddVisualization(image_visualization)


camera_radius = 5
camera_angle = 0
dt = 0.01
for t in np.arange(0, 10, dt):
    camera_angle += dt
    camera_position = chrono.ChVectorD(camera_radius * np.cos(camera_angle), camera_radius * np.sin(camera_angle), 2)
    camera_sensor.SetSensorOffset(camera_position)
    camera_sensor.SetSensorDirection(chrono.ChVectorD(0, 0, -1))
    camera_sensor.SetSensorUp(chrono.ChVectorD(0, 1, 0))

    
    my_system.DoStepDynamics(dt)
    print(camera_sensor.GetBufferData())


chrono.ChIrrApp().GetSystem().GetTimer().Stop()
chrono.ChIrrApp().GetSystem().CloseWindow()
chrono.ChIrrApp().GetSystem().RemoveAllLoggers()
chrono.ChIrrApp().GetSystem().Drop()