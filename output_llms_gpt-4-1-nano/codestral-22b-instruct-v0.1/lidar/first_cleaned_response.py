import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()



mesh_shape = chrono.ChTriangleMeshConnected()
mesh_shape.LoadWavefrontMesh('path_to_your_file.obj', True, True)
mesh_shape.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
mesh_material = chrono.ChMaterialSurfaceNSC()
mesh_body = chrono.ChBodyEasyMesh(mesh_shape, 1000, True, True, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), mesh_material)
my_system.Add(mesh_body)


lidar_sensor = sens.ChLidarSensor(mesh_body, 100, 100, 10, 1, 1, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
lidar_sensor.AddNoiseGaussian(0.01, 0.01)
lidar_sensor.SetVisualizeScanner(True)
lidar_sensor.SetSaveData(True)
my_system.AddSensor(lidar_sensor)



lidar_sensor.SetSensorOffsetPose(chrono.ChFrameD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))


application = chronoirr.ChIrrApp(my_system, 'Lidar Sensor Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 0, -3))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()

    
    time = my_system.GetChTime()
    lidar_sensor.SetSensorOffsetPose(chrono.ChFrameD(chrono.ChVectorD(2 * np.cos(time), 0, 2 * np.sin(time)), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))

    
    print(lidar_sensor.GetBuffer())

    application.EndScene()