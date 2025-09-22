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
camera = chronoirr.ChCamera(chronoirr.ChVectorD(0, 0, 1), chronoirr.ChVectorD(0, 0, 0))
camera.SetElevation(90)
scene.AddCamera(camera)


mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh("mesh.obj")


body = chrono.ChBodyEasyMesh(system, mesh, True, True)
body.SetMass(1)
body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(body)


lidar_manager = sensor.ChSensorManager(system)
lidar_manager.SetNoiseFilter(sensor.ChNoiseFilter.GAUSSIAN)
lidar_manager.SetVisualization(True)
lidar_manager.SetSaveData(True)


lidar = sensor.ChLidarSensor(system, body, chrono.ChVectorD(0, 0, 1), 10, 100, 10)
lidar_manager.AddSensor(lidar)


step_size = 0.01
sim_time = 10
for i in range(int(sim_time / step_size)):
    
    lidar.SetPos(chrono.ChVectorD(1 * chrono.cos(i * step_size), 1 * chrono.sin(i * step_size), 1))
    system.DoStepDynamics(step_size)
    
    print(lidar_manager.GetBufferData())


scene.SetSystem(system)
scene.SetCamera(camera)
scene.Run()