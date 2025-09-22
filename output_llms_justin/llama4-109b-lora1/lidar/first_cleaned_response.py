import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math


core = chrono.ChCore()
core.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
core.SetConstraintSystemType(chrono.ChSolver.Type_PENDANT)
core.SetSolverType(chrono.ChSolver.Type_PENDANT)
core.SetTimeIncrement(0.01)
core.SetTimestep(0.01)


manager = sens.ChSensorManager(core.GetSystem())


mesh = sens.ChTriangleMeshShape()
mesh.LoadWavefrontMesh('chrono_assets/cow.obj')
mesh.SetName('Mesh')
mesh.SetMutable(False)

body = chrono.ChBody()
body.SetFixed(True)
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.AddVisualShape(mesh)
core.GetSystem().AddBody(body)


lidar = sens.ChLidarSensor(body, 1, chrono.ChFrame3d(chrono.ChVector3d(0, 0, 1.6)), 0.03, 0.2, 0.02, 0.02, 0.02)
lidar.SetName('Lidar')
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.05)
lidar.SetPointCloudParameters(True, False, False)
lidar.SetRayHitReturnThickness(0.01)
lidar.SetRayHitReturnLength(True)
lidar.SetRayHitReturnNormal(True)


noise_range = sens.ChNoiseRangeLidar()
noise_range.SetName('NoiseRange')
noise_range.SetParameters(0.01, 0.01)
lidar.AddFilter(noise_range)

noise_gaussian = sens.ChNoiseGaussianLidar()
noise_gaussian.SetName('NoiseGaussian')
noise_gaussian.SetParameters(0.01, 0.01)
lidar.AddFilter(noise_gaussian)


vis = sens.ChLidarVisualizeSensor()
vis.SetName('LidarVisualize')
vis.SetParameters(0.01, 0.01, 0.01, 0.01)
lidar.AddFilter(vis)


manager.AddSensor(lidar)


manager.SetSaveSensors(False)
manager.SetVerbose(False)


angle = 0
angle_increment = 0.01

while True:
    
    lidar_pos = chrono.ChVector3d(5 * math.cos(angle), 5 * math.sin(angle), 1.6)
    lidar.SetFrame(chrono.ChFrame3d(lidar_pos, chrono.ChQuaterniond(1, 0, 0, 0)))
    angle += angle_increment

    
    core.DoStepDynamics(1)

    
    for buffer in manager.GetSensorManager().GetAllData():
        print(buffer)