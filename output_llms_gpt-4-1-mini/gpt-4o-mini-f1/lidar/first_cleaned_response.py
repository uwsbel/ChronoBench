import pychrono as chrono
import pychrono.sensor as sens
import math


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


mesh_body = chrono.ChBodyEasyMesh()
mesh_body.SetStatic(True)
mesh_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
mesh_body.GetVisualShape(0).SetMesh(chrono.GetChronoDataFile("meshes/triangulated.obj"))
sys.Add(mesh_body)


manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 10), chrono.ChColor(1, 1, 1), 500)


offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(mesh_body, 0.5, offset_pose, 90, 200, 2 * chrono.CH_PI, chrono.CH_PI / 6, -chrono.CH_PI / 6)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.1)
lidar.SetCollectionWindow(0.1)
lidar.SetSamplesPerPixel(1)
lidar.SetHorizontalResolution(800)
lidar.SetVerticalResolution(600)
lidar.SetFOVAngle(2 * chrono.CH_PI)
lidar.SetUpdateFrequency(10)
lidar.SetPointAccuracies(0.002, 0.002)
lidar.SetZClipPlane(0.1, 100)
lidar.SetNoiseModel(sens.ChLidarSensor.NoiseModelEnum_GAUSSIAN, 0.0, 0.01)
lidar.SetIntensityColors(sens.ChLidarSensor.IntensityColorType_GRAY)
lidar.SetImageType(sens.ChLidarSensor.ImageType_DEPTH)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1, "Lidar PC"))
lidar.PushFilter(sens.ChFilterSavePC(sens.GetChronoOutputPath() + "LidarPC.ply"))
manager.AddSensor(lidar)


noise = sens.ChLidarSensor.NoiseModelEnum_GAUSSIAN
lidar.SetNoiseModel(noise, 0.0, 0.01)


sys.Setup()
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.GetSolver().SetMaxIterations(50)
sys.GetSolver().SetTolerance(1e-6)


time_step = 1e-2
time = 0


while time < 10:
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(3 * math.cos(time), 3 * math.sin(time), 2), chrono.QuatFromAngleAxis(time, chrono.ChVector3d(0, 0, 1)))
    lidar.SetOffsetPose(offset_pose)
    
    
    print("Lidar buffer data:", lidar.GetCurrentDepthBuffer())
    
    
    manager.Update()
    
    
    sys.DoStepDynamics(time_step)
    time += time_step