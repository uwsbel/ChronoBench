import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh            
from  pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system.Set_G(chrono.ChVector3d(0, 0, -9.81))       


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




terrain = veh.SCMDeformableTerrain(system)


terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0),
                                    chrono.ChQuaterniond(1, 0, 0, 0)))

length   = 20.0     
width    = 20.0     
delta    = 0.04     

terrain.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0),
                                      chrono.ChQuaterniond(1, 0, 0, 0)),
                   length, width, delta)


terrain.SetSoilParametersBekker_Kphi_Kc_n(5.301e6, 102e3, 1.1)
terrain.SetSoilParametersMohrCoulomb(1.3, 30.0)
terrain.SetSoilParametersJanosiShear(0.01)
terrain.SetSoilParametersElastic_K(4.0e7)


terrain.EnableMovingPatch(True)




rover  = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(0.0, 0.20, 0.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))   




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover – SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0,   0,   0.5),
                       3, 4, 10, 40, 512)




time_step = 1e-3
driver.SetSteering(0.0)            




while vis.Run():

    system.DoStepDynamics(time_step)   
    rover.Update()                     

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()