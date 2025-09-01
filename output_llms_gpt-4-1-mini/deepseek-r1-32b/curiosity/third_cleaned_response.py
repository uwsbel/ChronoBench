import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))


manager = sens.ChSensorManager(system)  



lidar_update_rate = 10.0  
lidar_h_samples = 180  
lidar_v_samples = 16  
lidar_h_fov = 90.0  
lidar_v_fov = 30.0  
lidar_range = 10.0  
lidar_noise = 0.01  


lidar = sens.ChLidarSensor()
lidar.SetName("Rover Lidar")
lidar.SetUpdateRate(lidar_update_rate)
lidar.SetHSamples(lidar_h_samples)
lidar.SetVSamples(lidar_v_samples)
lidar.SetHFov(lidar_h_fov)
lidar.SetVFov(lidar_v_fov)
lidar.SetRange(lidar_range)
lidar.SetNoiseStandardDeviation(lidar_noise)


lidar.Attach(rover.GetChassis())


filters = sens.ChLidarFilterContainer()
filters.AddFilter(sens.ChLidarFilterRemoveGround())
filters.AddFilter(sens.ChLidarFilterVoxelGrid(0.1))  
lidar.SetFilterContainer(filters)


manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    manager.Update()  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)