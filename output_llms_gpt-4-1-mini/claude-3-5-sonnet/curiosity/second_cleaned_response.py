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
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  
ground.SetBodyFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)


obstacle_mat = chrono.ChMaterialSurfaceNSC()
obstacle = chrono.ChBodyEasyBox(1, 10, 0.5, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVectorD(0, 0, 0.25))  
obstacle.SetBodyFixed(True)  
system.Add(obstacle)


rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVectorD(-5, 0.0, 0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with obstacle')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, 3, 3), chrono.ChVectorD(-5, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step

    
    driver.SetSteering(0)  
    driver.SetThrottle(0.5)  

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)