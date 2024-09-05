import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3D(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChVector3D(0, 0, 1), 0)  
terrain.SetLength(20)  
terrain.SetWidth(20)  
terrain.SetResolution(100)  
terrain.SetSoilProperties(0.0, 0.0, 0.0, 0.0, 0.0)  
terrain.Initialize()


rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  


init_pos = chrono.ChVector3D(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 2.5, 1.5), chrono.ChVector3D(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3D(1.5, -2.5, 5.5), chrono.ChVector3D(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    time += time_step  
    driver.SetSteering(0.0)  

    rover.Update()  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)