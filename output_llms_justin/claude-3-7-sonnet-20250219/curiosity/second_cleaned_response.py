import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
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


obstacle_mat = chrono.ChMaterialSurfaceNSC()
obstacle = chrono.ChBodyEasyBox(1, 5, 0.2, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector(-2, 0, 0.1))  
obstacle.SetBodyFixed(True)  
system.Add(obstacle)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector(-5, 0.0, 0)  
init_rot = chrono.ChQuaternion(1, 0, 0, 0)
rover.Initialize(chrono.ChFrame(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with obstacle')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(-5, 3, 3), chrono.ChVector(-5, 0, 0))  
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector(1.5, -2.5, 5.5), chrono.ChVector(0, 0, 0), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    steering = 0  
    driver.SetSteering(steering)
    
    
    driver.SetThrottle(0.4)  

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)