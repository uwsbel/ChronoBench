import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


plane_length = 20.0
plane_width = 20.0
mesh_res_x = 0.2  
mesh_res_z = 0.2  
soil_density = 1500.0  
soil_cohesion = 10000.0  
soil_friction = 30.0 * math.pi / 180.0  
soil_adhesion = 0.0

terrain = veh.DeformableTerrainSCM(system)
terrain.SetPlaneLength(plane_length)
terrain.SetPlaneWidth(plane_width)
terrain.SetMeshResolutionX(mesh_res_x)
terrain.SetMeshResolutionZ(mesh_res_z)
terrain.SetSoilDensity(soil_density)
terrain.SetCohesion(soil_cohesion)
terrain.SetFrictionAngle(soil_friction)
terrain.SetAdhesion(soil_adhesion)
terrain.SetPos(chrono.ChVectorD(0, 0, -1))  
terrain.Initialize()  


rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)


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