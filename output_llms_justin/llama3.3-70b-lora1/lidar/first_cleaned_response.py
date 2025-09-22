import pychrono.core as chrono
import pychrono.sensor as sens

import time
import math
import numpy as np


sys = chrono.ChSystemNSC()


chrono.ChVisualSystem().SetCameraUpVector(chrono.ChVector3d(0, 1, 0))
chrono.ChVisualSystem().SetWindowSize(1024, 768)
chrono.ChVisualSystem().SetChaseCameraVerticalOffset(14)
chrono.ChVisualSystem().SetChaseCameraPoint(8, 0, 0)
chrono.ChVisualSystem().SetChaseCameraPosition(-25, 8, 2)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv_chassis.obj'), False, True)
mmesh = chrono.ChVisualShapeTriangleMesh()
mmesh.SetMesh(mesh)
mmesh.SetName('HMMWV Chassis Mesh')
mmesh.SetMutable(False)

trimesh_body = chrono.ChBody()
trimesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
trimesh_body.AddVisualShape(mmesh)
trimesh_body.SetFixed(True)
trimesh_body.SetRot(chrono.QuatFromAngleAxis(-chrono.CH_PI / 2, chrono.ChVector3d(1, 0, 0)))
sys.Add(trimesh_body)


update_rate = 30.0
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)))
lidar_push_frequency = 5  
max_angle = chrono.CH_PI
horizontal_samples = 450
vertical_samples = 1
min_fov = -max_angle
max_fov = max_angle
min_intensity = 0
max_intensity = 100
noise_none = False
noise_gaussian = False
visualize = True
save = False


noise_model_none = sens.ChNoiseNone()
noise_model_gaussian = sens.ChNoiseModelGaussian(0, 0.15, 0, 0.5)


manager = sens.ChSensorManager(sys)
lidar = sens.ChLidarSensor(
    trimesh_body,
    update_rate,  
    offset_pose,  
    horizontal_samples,  
    vertical_samples,  
    min_fov,  
    max_fov,  
    -chrono.CH_PI / 2,  
    chrono.CH_PI / 2,  
    200,  
    1.0  
)

if (noise_none):
    lidar.PushFilter(sens.ChFilterLidarNoise(None))  
if (noise_gaussian):
    lidar.PushFilter(sens.ChFilterLidarNoise(gaussian_noise))  
if (visualize):
    lidar.PushFilter(sens.ChFilterVisualize(
        0,  
        0,  
        200,  
        255  
    ))
if (save):
    lidar.PushFilter(sens.ChFilterSave("lidar/data"))

manager.AddSensor(lidar)
pulse_time = 0
radius = 10
height = 2
manager.Update()
while (sys.GetChTime() < 20):

    time = sys.GetChTime()

    
    radius = 10 + 5 * math.sin(2 * time)

    
    lidar.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(radius * math.cos(time), radius * math.sin(time), height),
        chrono.QuatFromAngleAxis(time, chrono.ChVector3d(0, 1, 0))
    ))

    
    manager.Update()
    if (int(lidar.HasData())):
        lidar_Ab = lidar.GetLidarPointCloud()
        print('---------------------------')
        print('Lidar buffer data:')
        print('Access lidar point cloud data using GetLidarPointCloud()')
        print('  GetPointCloud() returns a list of all return points')
        print('  GetAccessPointCloud() returns a list of all return points, including access to intensity')
        print('  GetDepth() returns a depth buffer')
        print('  GetRGB() returns a rgb buffer')
        print('  GetCameraToLidarTransform() returns the camera to lidar transform')
        print('  GetMaxHorizontalFOV() returns the max horizontal field of view')
        print('  GetMaxVerticalFOV() returns the max vertical field of view')
        print('  GetMaxDepth() returns the max depth')
        print('  GetMinDepth() returns the min depth')
        print('  GetIntensityScale() returns the intensity scale')
        print('  GetIntensityOffset() returns the intensity offset')
        print('  GetHorizontalSamples() returns the number of horizontal samples')
        print('  GetVerticalSamples() returns the number of vertical samples')
        print('  GetUpdateRate() returns the update rate')
        print('  GetLidarPose() returns the lidar pose')
        print('  GetPointCloudAccess() returns the point cloud access')
        print('  GetDepthAccess() returns the depth access')
        print('  GetRGBAcess() returns the rgba access')
        print('  GetIntensityAccess() returns the intensity access')
        print('  GetCameraPose() returns the camera pose')
        print('  GetCameraFOV() returns the camera fov')
        print('  GetCameraAspect() returns the camera aspect ratio')
        print('---------------------------')

    sys.DoStepDynamics(1e-3)