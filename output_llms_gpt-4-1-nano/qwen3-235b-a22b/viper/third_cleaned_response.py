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
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
terrain.SetLength(20)    
terrain.SetWidth(20)     
terrain.SetMeshResolution(0.1)  


soil_params = veh.SCMSoilParams()
soil_params.BekkerKphi = 0.2e6    
soil_params.BekkerKc = 0          
soil_params.Bekkern = 1.1         
soil_params.MohrCoulombCohesion = 0
soil_params.MohrCoulombFrictionAngle = 30 * chrono.CH_PI / 180  
soil_params.DrLateralFriction = 0.03
soil_params.YoungsModulus = 2e7   
soil_params.PoissonsRatio = 0.3   
soil_params.plastic_flow = True
soil_params.use_shear = True
terrain.SetSoilParameters(soil_params)
terrain.Initialize()  


rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  


init_pos = chrono.ChVector3d(0, 0, 0.2)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - SCM Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.5))
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