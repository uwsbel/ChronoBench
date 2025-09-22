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
lidar_sensor = sens.ChLidarSensor(mesh_body, 1000, 100, 1, 20, 0.05, 10, 0.1, 10)
lidar_manager.AddSensor(lidar_sensor)


lidar_sensor.SetFilter(sens.ChSensorFilterNoiseGaussian(0.01))
lidar_sensor.SetVisualizationType(sens.ChSensorVisualizationType_POINTS)
lidar_sensor.SetDataSaving(True)


lidar_radius = 5
lidar_angle = 0


application = irr.ChIrrApp(my_system, 'Lidar sensor simulation', irr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(irr.vector3df(0, 0, -lidar_radius), irr.vector3df(0, 0, 0))
application.SetTimestep(0.01)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    lidar_angle += 0.01
    lidar_x = lidar_radius * np.cos(lidar_angle)
    lidar_y = lidar_radius * np.sin(lidar_angle)
    lidar_sensor.SetSensorOffsetPose(chrono.ChFrameD(chrono.ChVectorD(lidar_x, lidar_y, 0), chrono.Q_from_AngX(np.pi/2)))

    
    lidar_data = lidar_sensor.GetData()
    print(lidar_data.buffer)

    application.DoStep()
    application.EndScene()

application.GetDevice().closeDevice()