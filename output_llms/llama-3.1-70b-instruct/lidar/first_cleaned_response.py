import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))
system = chrono.ChSystemNSC()


visualize = True
if visualize:
    app = chronoirr.ChVisualSystemIrrlicht()
    app.SetWindowSize(800, 600)
    app.SetWindowTitle("Lidar simulation")
    app.SetCameraDistance(2.5)
    app.SetCameraAzimuth(1.57)
    app.SetCameraElevation(0.5)


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh("mesh.obj")


body = chrono.ChBodyEasyMesh(mesh, 1000, True, False)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetBodyFixed(True)
system.Add(body)


lidar = sens.ChLidarSensor(body, 10, 0.1, 10, 0.1, 100, 100)
lidar.SetName("LidarSensor")
lidar.SetNoiseType(sens.ChLidarSensor.NOISE_GAUSSIAN)
lidar.SetNoiseParams(0.01, 0.01)
lidar.SetVisualizationMode(sens.ChLidarSensor.VISUALIZATION_WIREFRAME)
lidar.SetVisualizationColor(chrono.ChVectorD(1, 0, 0))
lidar.SetSaveDataMode(sens.ChLidarSensor.SAVE_DATA_BUFFER)
lidar.SetSaveDataFilename("lidar_data.csv")
body.AddSensor(lidar)


sensor_manager = sens.ChSensorManager(system)
sensor_manager.AddSensor(lidar)


time_step = 0.01
while system.GetChTime() < 10:
    system.DoStepDynamics(time_step)
    sensor_manager.UpdateSensors()
    lidar_buffer = lidar.GetBuffer()
    print(lidar_buffer)
    if visualize:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        app.Step()


system.GetCollisionSystem().Clear()