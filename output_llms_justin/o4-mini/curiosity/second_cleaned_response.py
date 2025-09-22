import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)





static_mat = chrono.ChMaterialSurfaceNSC()

ground = chrono.ChBodyEasyBox(
    20,   
    20,   
    1,    
    1000, 
    True, 
    True, 
    static_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




obst_mat = chrono.ChMaterialSurfaceNSC()
obstacle = chrono.ChBodyEasyBox(
    5.0,  
    1.0,  
    0.5,  
    1000, 
    True, 
    True, 
    obst_mat)

obstacle.SetPos(chrono.ChVector3d(0, 0, 0.25))
obstacle.SetFixed(True)
system.Add(obstacle)




rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(-5.0, 0.0, 0.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

rover.Initialize(chrono.ChCoordsysD(init_pos, init_rot))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover – obstacle crossing')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-2, -6, 3),    
              chrono.ChVector3d(0, 0, 0))      
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(5, -5, 7),
    chrono.ChVector3d(0, 0, 0),
    3, 4, 10, 40, 512)






time_step = 1e-3
driver.SetThrottle(1.0)  
driver.SetSteering(0.0)  




while vis.Run():
    rover.Update()             
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)