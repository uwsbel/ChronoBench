import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

Create Chrono system
===================

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

Create SCM deformable terrain
============================

terrain_params = veh.SCMDeformableTerrain.SCM_SOIL_PARAMS()
terrain_params.mu = 0.5  
terrain_params.density = 1500  
terrain_params.Ks = 5e6  
terrain_params.Kv = 1e6  
terrain_params.Kp = 5e4  

terrain = veh.SCMDeformableTerrain(system, chrono.ChVector3d(0, 0, -1), chrono.ChVector3d(10, 10, 0.1),  
                                   100, 100, terrain_params)  

Create Viper rover
==================

rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  

Initialize rover position and orientation
===========================================

init\_pos = chrono.ChVector3d(0, 0.2, 0)
init\_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init\_pos, init\_rot))

Create run-time visualization
============================

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir\_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo\_pychrono\_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)




Set the simulation time step
===========================

time\_step = 1e-3

Simulation loop
==============

time = 0
while vis.Run():
time += time\_step  
steering = 0.0  

driver.SetSteering(steering)  

rover.Update()  

vis.BeginScene()
vis.Render()
vis.EndScene()

system.DoStepDynamics(time\_step)