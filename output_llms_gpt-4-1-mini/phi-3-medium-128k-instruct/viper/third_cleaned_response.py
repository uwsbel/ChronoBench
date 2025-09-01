import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh  


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain_params = chrono.ChTerrainSCMParams()
terrain_params.SetTerrainType(chrono.ChTerrainSCMType.Terrain_Plane)
terrain_params.SetLength(10)
terrain_params.SetWidth(10)
terrain_params.SetMeshResolution(0.1)
terrain_params.SetSoilParams(0.5, 0.5, 0.5, 0.5, 0.5, 0.5)
terrain = chrono.ChTerrainSCM(terrain_params)
terrain.SetFixed(True)  


rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    time += time_step  
    steering = 0.0  
    
    driver.SetSteering(steering)  
    rover.Update()  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)