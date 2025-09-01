import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1), chrono.QUNIT))  
terrain.Initialize(20, 20, 0.04)  


soil_params = veh.SCMDeformableTerrain.SoilParameters()
soil_params.m_Bekker_Kphi = 0.68e6  
soil_params.m_Bekker_Kc = 1.37e4   
soil_params.m_Bekker_n = 0.85      
soil_params.m_Mohr_cohesion = 1.3e3  
soil_params.m_Mohr_friction = 30   
soil_params.m_Janosi_shear = 0.01  
soil_params.m_elastic_K = 4e7      
soil_params.m_damping_R = 3e4      

terrain.SetSoilParameters(soil_params)


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
vis.SetWindowTitle('Viper rover - SCM deformable terrain')
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
    terrain.Synchronize(time)  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)