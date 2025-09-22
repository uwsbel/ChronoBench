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



ground_mat = chrono.ChMaterialSurfaceNSC()




ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


obs_dims = chrono.ChVector3d(0.5, 4.0, 0.2) 
obs_pos = chrono.ChVector3d(0, 0, obs_dims.z / 2.0) 

obstacle_mat = chrono.ChMaterialSurfaceNSC() 


obstacle = chrono.ChBodyEasyBox(obs_dims.x, obs_dims.y, obs_dims.z, 1000, True, True, obstacle_mat)
obstacle.SetPos(obs_pos)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.7)) 
system.Add(obstacle)



rover = robot.Curiosity(system) 


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)





init_pos = chrono.ChVector3d(-5, 0.0, 0.3)

init_rot = chrono.ChQuaterniond(1, 0, 0, 0) 
rover.Initialize(chrono.ChFramed(init_pos, init_rot))





driver.SetSteering(0.0) 

forward_voltage = 10.0  
                        
                        
for i in range(6):
    driver.SetMotorVoltage(forward_voltage, i)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Obstacle crossing')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

vis.AddCamera(chrono.ChVector3d(-7, 3, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    current_time = system.GetChTime() 

    
    

    
    rover.Update()

    
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) 
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    
    

    
    if current_time > 15:
        vis.GetDevice().closeDevice()


delপ্রেম