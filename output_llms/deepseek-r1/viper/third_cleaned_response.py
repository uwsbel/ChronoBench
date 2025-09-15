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
terrain.SetPlane(veh.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.Initialize(20, 20, 0.1)  


Kphi = 0.82e6  
Kc = 0.14e4    
n = 1.0         
coh = 3e4       
mu = 0.3        
H = 0.2         
m = 0.2         
Kn = 4e6        
Gn = 5e7        
Kt = 2e4        
Gt = 5e4        

terrain.SetSoilParameters(Kphi, Kc, n, coh, mu, H, m, Kn, Gn, Kt, Gt)


terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
texture_file = chrono.GetChronoDataFile("textures/grass.jpg")
terrain.SetTexture(texture_file, 20, 20)


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