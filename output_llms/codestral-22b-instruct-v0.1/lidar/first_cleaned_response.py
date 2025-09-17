import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath("PYCHRONO_DATA_DIR")
my_system = chrono.ChSystemNSC()


mesh_shape = chrono.ChTriangleMeshConnected()
mesh_shape.LoadWavefrontMesh("mesh.obj")
mesh_material = chrono.ChMaterialSurfaceNSC()
mesh_body = chrono.ChBodyEasyMesh(mesh_shape, 1000, True, True, mesh_material)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)
my_system.Add(mesh_body)


lidar_manager = sens.ChSensorManager(my_system)
lidar_sensor = sens.ChLidarSensor(mesh_body, 1000, 100, 1, 20, 10, 1, 0.01, 0.1, 0.5, 100)
lidar_sensor.AddFilter(sens.ChFilterAccessRecorder())
lidar_sensor.AddFilter(sens.ChFilterNoise(0.01))
lidar_sensor.AddFilter(sens.ChFilterOutliers(0.02))
lidar_sensor.AddFilter(sens.ChFilterRadius(0.05))
lidar_sensor.AddFilter(sens.ChFilterSmoother(10))
lidar_sensor.AddVisualizationType(sens.ChVisualizationTypeSensor(lidar_sensor))
lidar_sensor.AddDataRecorder(sens.ChDataRecorder(lidar_sensor))
lidar_manager.AddSensor(lidar_sensor)


lidar_radius = 2
lidar_angle = 0
lidar_angle_increment = 0.01

for i in range(1000):
    lidar_x = lidar_radius * np.cos(lidar_angle)
    lidar_y = lidar_radius * np.sin(lidar_angle)
    lidar_sensor.SetSensorPose(chrono.ChFrameD(chrono.ChVectorD(lidar_x, lidar_y, 1), chrono.Q_from_AngAxis(np.pi/2, chrono.ChVectorD(0, 0, 1))))
    my_system.DoStepDynamics(0.01)
    lidar_data = lidar_sensor.GetSensorData()
    print(lidar_data.buffer)
    lidar_angle += lidar_angle_increment