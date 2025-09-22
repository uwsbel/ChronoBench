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


terrain_shape = veh.SCMDeformableTerrain(system)
terrain_shape.SetPlane(chrono.ChCoordsys3d(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngZ(0)))
terrain_shape.SetSize(20, 20)
terrain_shape.SetMeshResolution(0.25, 0.25)
terrain_shape.Initialize()


terrain_mat = veh.SCMTerrainMaterial(2e4,  
                                      5e6,  
                                      0.3,   
                                      0,     
                                      30,    
                                      10,    
                                      0.01,  
                                      0)     
terrain_shape.SetMaterial(0, 0, terrain_mat)
terrain_shape.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
terrain_shape.SetContactFrictionCoefficient(0.9)
terrain_shape.SetContactRestitutionCoefficient(0.01)


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