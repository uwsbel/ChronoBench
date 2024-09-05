import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
from pychrono import sensor as sens


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
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)





time_step = 1e-3


manager = sens.ChSensorManager(system)


lidar = sens.ChLidarSensor(system)
lidar.SetUpdateRate(10)  
lidar.SetHorizontalSamples(180)
lidar.SetVerticalSamples(180)
lidar.SetFieldOfView(chrono.ChVector3d(0, 0, 180))
lidar.SetMaxRange(10)
lidar.SetMinRange(0.1)
lidar.SetMaxAngle(chrono.ChVector3d(0, 0, 180))
lidar.SetMinAngle(chrono.ChVector3d(0, 0, -180))
lidar.SetSensorPosition(chrono.ChVector3d(0, 0, 0))
lidar.SetSensorOrientation(chrono.ChQuaterniond(1, 0, 0, 0))
lidar.SetSensorHeight(0.2)


filter = sens.ChLidarFilter(lidar)
filter.SetFilterType(sens.LidarFilterType_GAUSSIAN)
filter.SetSigma(1.0)


manager.AddSensor(lidar)
manager.AddFilter(filter)


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


vis.ShutDown()