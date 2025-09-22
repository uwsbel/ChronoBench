import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens

import numpy as np
import random
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,0))


ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


emitter = sens.ChParticleEmitter()
emitter.SetEmissionRate(1)
emitter.SetParticlesPerEmission(1)
emitter.SetParticleSpeedMean(1)
emitter.SetParticleSpeedSigma(0.2)
emitter.SetParticleSizeMean(0.1)
emitter.SetParticleSizeSigma(0.01)


manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(100,100,100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100,-100,100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100,100,-100),chrono.ChColor(intensity,intensity,intensity),1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(100,-100,-100),chrono.ChColor(intensity,intensity,intensity),1000.0)


filter_graph_1 = sens.ChFilterGraph()
filter_graph_1.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 1"))
filter_graph_1.SetName("Filter Graph 1")


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
cam = sens.ChCameraSensor(
    ground,                  
    update_rate,             
    offset_pose,             
    image_width,             
    image_height,            
    fov                    
)
cam.SetName("Camera Sensor 1")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
manager.AddSensor(cam)
manager.SetFilterGraph(cam,filter_graph_1)


filter_graph_2 = sens.ChFilterGraph()
filter_graph_2.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 2"))
filter_graph_2.SetName("Filter Graph 2")


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
cam = sens.ChCameraSensor(
    ground,                  
    update_rate,             
    offset_pose,             
    image_width,             
    image_height,            
    fov                    
)
cam.SetName("Camera Sensor 2")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)
manager.AddSensor(cam)
manager.SetFilterGraph(cam,filter_graph_2)


filter_graph_3 = sens.ChFilterGraph()
filter_graph_3.AddFilter(sens.ChFilterVisualize(1280,720,"Filter Graph 3"))
filter_graph_3.SetName("Filter Graph 3")


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
depth_cam = sens.ChDepthCamera(
    ground,                  
    update_rate,             
    offset_pose,             
    image_width,             
    image_height,            
    fov                    
)
depth_cam.SetName("Depth Camera Sensor")
depth_cam.SetLag(lag)
depth_cam.SetCollectionWindow(exposure_time)
depth_cam.SetDepthRange(0.1,10.0)
depth_cam.SetDepthQuantization(1e-3)
manager.AddSensor(depth_cam)
manager.SetFilterGraph(depth_cam,filter_graph_3)


filter_graph_4 = sens.ChFilterGraph()
filter_graph_4.AddFilter(sens.ChFilterVisualizePointCloud(1280,720,"Filter Graph 4"))
filter_graph_4.SetName("Filter Graph 4")


offset_pose = chrono.ChFramed(chrono.ChVector3d(-5,0,2))
lidar = sens.ChLidarSensor(
    ground,                  
    update_rate,             
    offset_pose,             
    horizontal_samples,      
    vertical_samples,        
    horizontal_fov,          
    max_vert_angle,
    min_vert_angle,
    100 
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(exposure_time)
lidar.SetReturnMode(sens.ChLidarReturnMode_STRONGEST_RETURN)
manager.AddSensor(lidar)
manager.SetFilterGraph(lidar,filter_graph_4)


orbit_radius = 5
orbit_rate = 0.2
ch_time = 0.0

while (ch_time < end_time):

    
    time = ch_time / orbit_rate
    cam.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(time), -orbit_radius * math.sin(time), 1),
        chrono.QuatFromAngleAxis(time, chrono.ChVector3d(0,0,1)))
    )

    
    accessacam1 = manager.GetMostRecentSample(cam)
    if (accessacam1.HasData()):
        rgba8_buffer = accessacam1.GetRGBA8Buffer()
        print('Camera 1 Resolution: ', accessacam1.ImageWidth, accessacam1.ImageHeight)
        print('Camera 1 Data Size: ', len(rgba8_buffer))

    accessacam2 = manager.GetMostRecentSample(cam)
    if (accessacam2.HasData()):
        rgba8_buffer = accessacam2.GetRGBA8Buffer()
        print('Camera 2 Resolution: ', accessacam2.ImageWidth, accessacam2.ImageHeight)
        print('Camera 2 Data Size: ', len(rgba8_buffer))

    accesslidar = manager.GetMostRecentSample(lidar)
    if (accesslidar.HasData()):
        float32_buffer = accesslidar.GetDepthBuffer()
        print('Lidar Resolution: ', accesslidar.ImageWidth, accesslidar.ImageHeight)
        print('Lidar Data Size: ', len(float32_buffer))

    
    
    manager.Update()

    
    step_size = 1e-3
    system.DoStepDynamics(step_size)

    
    ch_time = system.GetChTime()