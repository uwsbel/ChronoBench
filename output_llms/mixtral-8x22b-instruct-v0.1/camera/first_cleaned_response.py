import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as chronopost
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'chrono_data'))
chrono.SetChronoVisualization(chronoirr.ChVisualizationIRRLICHT)


my_system = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float64)
indices = np.array([[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]], dtype=np.uint32)
mesh.LoadWavefront(vertices, indices)


body = chrono.ChBodyEasyBox(2, 2, 2, 1000, True, True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.AddAsset(chrono.ChTriangleMeshShape(mesh))
my_system.Add(body)


camera_sensor = chrono.ChCameraSensor(chrono.ChVectorD(2, 0, 0), chrono.ChVectorD(0, 0, 0), 45, 1, 100)
camera_sensor.SetName("Camera Sensor")


sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(camera_sensor)


noise_filter = chrono.ChFilterAccessSensor(camera_sensor)
noise_filter.Set_noise_type(chrono.ChFilterAccessSensor.FILTER_NOISE_GAUSSIAN)
noise_filter.Set_noise_value(0.01)
visualization = chrono.ChVisualizationSensor(camera_sensor)


my_system.Add(sensor_manager)


camera_sensor.SetPos(chrono.ChVectorD(2, 0, 0))
camera_sensor.SetDir(chrono.ChVectorD(0, 0, 0))


camera_orbit_radius = 3
camera_orbit_speed = 0.01
camera_orbit_angle = 0


for step in range(1000):
    
    camera_orbit_angle += camera_orbit_speed
    camera_sensor.SetPos(chrono.ChVectorD(camera_orbit_radius * np.cos(camera_orbit_angle),
                                          camera_orbit_radius * np.sin(camera_orbit_angle),
                                          0))
    camera_sensor.SetDir(chrono.ChVectorD(0, 0, 0))

    
    my_system.DoStepDynamics(0.01)

    
    print(f"Step: {step}")
    print(f"Camera Buffer Data: {camera_sensor.GetBufferData()}")