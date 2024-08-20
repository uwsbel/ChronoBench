import pychrono.core as chrono
import pychrono.sensor as chrono_sensor
import pychrono.irrlicht as chrono_irrlicht
import numpy as np


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


scene = chrono_irrlicht.ChVisualSystemIrrlicht()
scene.SetWindowSize(800, 600)
scene.SetWindowTitle('PyChrono Lidar Simulation')
scene.SetLogoFile('data/logo.png')
scene.SetCameraDistance(2.0)
scene.SetCameraAzimuth(0.0)
scene.SetCameraElevation(0.0)


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh('data/mesh.obj')
mesh.SetName('Mesh')


body = chrono.ChBodyEasyMesh(mesh, 1000, True, True)
body.SetName('Body')
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(body)


lidar = chrono_sensor.ChLidarSensor()
lidar.SetName('Lidar')
lidar.SetFrequency(10.0)
lidar.SetRange(10.0)
lidar.SetBeamCount(64)
lidar.SetHorizontalFOV(np.pi / 2)
lidar.SetVerticalFOV(np.pi / 4)
lidar.SetNoiseModel(chrono_sensor.ChLidarNoiseModel.GAUSSIAN)
lidar.SetNoiseParams(0.01, 0.01)
lidar.SetVisualization(True)
lidar.SetSaveData(True)
lidar.SetSavePath('data/lidar_data.csv')
body.AddSensor(lidar)


sensor_manager = chrono_sensor.ChSensorManager()
sensor_manager.AddSensor(lidar)
system.Add(sensor_manager)


system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)
system.SetTimestep(0.01)


while system.GetChTime() < 10.0:
    
    lidar.SetPos(chrono.ChVectorD(np.cos(system.GetChTime()), np.sin(system.GetChTime()), 1.0))
    lidar.SetRot(chrono.ChQuaternionD(np.cos(system.GetChTime() / 2), np.sin(system.GetChTime() / 2), 0, 0))

    
    system.DoStepDynamics(system.GetTimestep())

    
    print(lidar.GetBuffer())

    
    scene.Render()
    scene.Synchronize(system.GetChTime())


system.SaveData('data/simulation_data.csv')