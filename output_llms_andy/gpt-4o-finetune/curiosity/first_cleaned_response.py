import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as crobot
from pychrono import irrlicht as chronoirr

def main():
    
    
    system = chrono.ChSystemNSC()
    
    
    
    phys_mat = chrono.ChContactMaterialNSC()
    phys_mat.SetFriction(0.9)  
    phys_mat.SetRestitution(0.01)  

    
    
    ground = chrono.ChBodyEasyBox(20, 20, 1,  
                                  phys_mat,   
                                  1000,       
                                  True,       
                                  True)       
    
    ground.SetPos(chrono.ChVector3d(0, 0, -1))  
    ground.SetRot(chrono.QUNIT)  
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    
    ground.SetFixed(True)  
    system.Add(ground)  

    
    
    rover = crobot.CuriosityMarsRover(system)
    rover.Initialize()  

    
    
    driver = crobot.CuriosityMotorControl()
    rover.SetDriver(driver)  

    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)  
    vis.SetCameraVertical(chrono.CameraVerticalDir.Z)  
    vis.SetWindowSize(1280, 720)  
    vis.SetWindowTitle('Curiosity rover - Rigid terrain')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))  
    vis.AddTypicalLights()  
    vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)  
    vis.EnableShadowPlots()  

    
    time_step = 1e-3  

    
    time = 0
    time_limit = 30  

    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    while vis.Run():
        time = rover.GetChTime()  

        
        if time > time_limit:
            break

        
        driver.SetSteering(0.3 * math.sin(0.2 * time))  
        rover.Update()  

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()