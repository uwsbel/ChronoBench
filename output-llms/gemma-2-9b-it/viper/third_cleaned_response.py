import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
from pychrono.vehicle import *


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
terrain.SetPos(chrono.ChVector3d(0, 0, -1))  
terrain.SetFixed(True)  
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.8)
terrain.GetMaterialSurface().SetRestitution(0.1)


terrain_params = chrono.ChSCMDeformableTerrainParams()
terrain_params.SetPlane(True)
terrain_params.SetLength(20)
terrain_params.SetWidth(20)
terrain_params.SetMeshResolution(10)
terrain_params.SetSoilParameters(
    density=1000,
    young_modulus=1e6,
    poisson_ratio=0.3,
    cohesion=100,
    friction_angle=math.radians(30),
)


terrain.SetSCMDeformableTerrainParams(terrain_params)


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
steering = 0.0  

while vis.Run():
    time += time_step  
    
    driver.SetSteering(steering)  

    rover.Update()  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)