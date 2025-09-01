import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as robot
from   pychrono import irrlicht as chronoirr





system = chrono.ChSystemNSC()


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))                       
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(2.5e-3)
chrono.ChCollisionModel.SetDefaultSuggestedMargin  (2.5e-3)





ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20.0, 20.0, 1.0,        
                              1000,                   
                              True, True, ground_mat) 
ground.SetPos  (chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)





obs_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(1.0,           
                                5.0,           
                                0.30,          
                                800,           
                                True, True, obs_mat)
obstacle.SetPos  (chrono.ChVector3d(0.0, 0.0, 0.15))   
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.2, 0.2))
system.Add(obstacle)





rover  = robot.Curiosity(system)               
driver = robot.CuriosityDCMotorControl()       
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(-5.0, 0.0, 0.0)   
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    
rover.Initialize(chrono.ChCoordsysD(init_pos, init_rot))   






driver.SetThrottle(0.6)    
driver.SetSteering(0.0)    





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)

vis.SetWindowSize (1280, 720)
vis.SetWindowTitle('Curiosity rover – rigid terrain with obstacle')
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()

vis.AddLogo (chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d( 1.5, -2.5,  5.5),
                       chrono.ChVector3d( 0.0,  0.0,  0.0),
                       3, 4, 10, 40, 512)






time_step = 1e-3      
simulation_time = 0.0

while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    rover.Update()

    
    system.DoStepDynamics(time_step)
    simulation_time += time_step