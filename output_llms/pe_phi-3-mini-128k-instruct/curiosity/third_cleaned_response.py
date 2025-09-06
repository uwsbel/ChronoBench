import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  


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



time_step = 1e-3


sensor_manager = sens.ChSensorManager(system)
lidar_params = sens.ChLidarSensorParams()
lidar_params.SetUpdateRate(10)  
lidar_params.SetHorizontalSamples(180)
lidar_params.SetVerticalSamples(270)
lidar_params.SetFieldOfView(15)  
lidar_params.SetMaxRange(100)  
lidar_params.SetNearPlane(0.01)  
lidar_params.SetFarPlane(100)  
lidar_params.SetNearPlaneResolution(0.01)  
lidar_params.SetFarPlaneResolution(0.01)  
lidar_params.SetNearPlaneFilterSize(1.0)  
lidar_params.SetFarPlaneFilterSize(10.0)  
lidar_sensor = sens.ChLidarSensor(lidar_params)
lidar_sensor.AttachTo(rover.GetLink(rover.GetLink(0))  
sensor_manager.Add(lidar_sensor)


time = 0
while vis.Run():
    time += time_step

    
    sensor_manager.Update()

    
    driver.SetSteering(0.0)

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)