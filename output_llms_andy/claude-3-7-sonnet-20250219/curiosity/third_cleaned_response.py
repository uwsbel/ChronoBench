import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChMaterialSurfaceNSC()  
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector(0, 0, -0.5))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector(0, 0, 0.0))  
box.SetBodyFixed(True)  
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector(-5, 0.0, 0)  
init_rot = chrono.ChQuaternion(1, 0, 0, 0)  
rover.Initialize(chrono.ChFrame(init_pos, init_rot))  


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)


lidar_update_rate = 10.0  
lidar_horizontal_samples = 1000
lidar_vertical_samples = 16
lidar_horizontal_fov = 2 * chrono.CH_C_PI  
lidar_vertical_fov = chrono.CH_C_PI / 12  
lidar_max_range = 100.0  
lidar_min_range = 0.1  


rover_body = rover.GetChassisBody()


lidar_offset_pose = chrono.ChFrame(chrono.ChVector(0, 0, 1.0), chrono.ChQuaternion(1, 0, 0, 0))


lidar = sens.ChLidarSensor(
    rover_body,              
    lidar_update_rate,       
    lidar_offset_pose,       
    lidar_horizontal_samples,
    lidar_vertical_samples,  
    lidar_horizontal_fov,    
    lidar_vertical_fov,      
    lidar_min_range,         
    lidar_max_range          
)


lidar.SetLidarNoise(sens.ChLidarNoiseNone())


lidar_vis = sens.ChFilterVisualize(1280, 720, "Lidar")
lidar.AddFilter(lidar_vis)


point_cloud_vis = sens.ChFilterVisualizePointCloud(1280, 720, "Lidar Point Cloud", 0.1)
lidar.AddFilter(point_cloud_vis)


manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(0, 3, 3), chrono.ChVector(0, 0, 0))  
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(1.5, -2.5, 5.5), 3, chrono.ChColor(0.7, 0.7, 0.7))  


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    driver.SetThrottle(0.5)  
    driver.SetSteering(0.0)

    
    rover.Update()
    
    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)