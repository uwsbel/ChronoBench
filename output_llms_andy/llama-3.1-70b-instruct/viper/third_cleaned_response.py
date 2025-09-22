import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr
from pychrono import vehicle as veh


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain_length = 10.0
terrain_width = 10.0
terrain_mesh_res = 0.1
terrain_soil_type = 0  
terrain_soil_par = veh.SoilParameters()
terrain_soil_par.kC = 2e6  
terrain_soil_par.kPhi = 0.5  
terrain_soil_par.kPsi = 0.5  
terrain_soil_par.kE = 1e6  
terrain_soil_par.knu = 0.3  
terrain_soil_par.kcohesion = 2e6  
terrain_soil_par.kadhesion = 0.0  
terrain_soil_par.kfriction = 0.5  
terrain_soil_par.kdensity = 1500  
terrain_soil_par.kyoungs = 2e6  
terrain_soil_par.kpoisson = 0.3  
terrain_soil_par.kadhesion = 0.0  
terrain_soil_par.krolling = 0.0  
terrain_soil_par.kspring = 1e6  
terrain_soil_par.kdamping = 0.0  
terrain_soil_par.kmu1 = 0.5  
terrain_soil_par.kmu2 = 0.5  
terrain_soil_par.kCte = 0.0  
terrain_soil_par.kCteI = 0.0  
terrain_soil_par.kCteV = 0.0  
terrain_soil_par.kCteL = 0.0  
terrain_soil_par.kCteR = 0.0  
terrain_soil_par.kCteF = 0.0  
terrain_soil_par.kCteB = 0.0  
terrain_soil_par.kCteT = 0.0  
terrain = veh.SCMDeformableTerrain(system, terrain_length, terrain_width, terrain_mesh_res, terrain_soil_type, terrain_soil_par)
terrain.SetName("terrain")
terrain.SetCollide(True)
system.Add(terrain)


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
vis.SetWindowTitle('Viper rover - Deformable terrain')
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