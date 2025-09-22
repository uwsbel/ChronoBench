import pychrono.core as chrono
import pychrono.sensor as sensor
import pychrono.irrlicht as chronoirr
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))
system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetMaxItersSolverSpeed(100)
system.SetMinBounceSpeed(1e-5)


scene = chronoirr.ChVisualSystemIrrlicht()
scene.SetWindowSize(800, 600)
scene.SetWindowTitle("Lidar Simulation")
scene.AddTypicalLights()
camera = chronoirr.ChCamera()
camera.SetPosition(chrono.ChVectorD(0, 0, 3))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))
scene.AddCamera(camera)


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh("mesh.obj")
mesh.SetStatic(True)


body = chrono.ChBodyEasyMesh(mesh, 1000, True, True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(body)


lidar = sensor.ChLidarSensor(body, 10, 10, 10, 10, 10, 10)
lidar.SetName("Lidar")
lidar.SetNoiseType(sensor.ChLidarSensor.NoiseType_GAUSSIAN)
lidar.SetNoiseParams(0.01, 0.01)
lidar.SetVisualizationMode(sensor.ChLidarSensor.VisualizationMode_RAYCAST)
lidar.SetSaveDataMode(sensor.ChLidarSensor.SaveDataMode_BUFFER)
lidar.SetBufferSize(100)
system.Add(lidar)


sensor_manager = sensor.ChSensorManager(system)
sensor_manager.AddSensor(lidar)


step = 0
while step < 100:
    
    angle = step * 0.01
    lidar.SetPos(chrono.ChVectorD(2 * chrono.cos(angle), 2 * chrono.sin(angle), 0))
    
    
    system.DoStepDynamics(0.01)
    
    
    print("Lidar Buffer Data:")
    print(lidar.GetBuffer())
    
    
    scene.Render()
    step += 1


sensor_manager.SaveData()