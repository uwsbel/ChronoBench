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












terrain = chrono.SCMDeformableTerrain(system)



terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))






terrain.SetSoilParameters(
    0.2e6,  
    0,      
    1.1,    
    20e3,   
    30,     
    0.01,   
    2e8,    
    0.3     
)


terrain_length = 20.0  
terrain_width = 20.0   
mesh_resolution = 0.4  
terrain.Initialize(terrain_length, terrain_width, mesh_resolution)





terrain.SetPlotType(chrono.SCMDeformableTerrain.PLOT_MESH, "SCM_Mesh", 0.0)

terrain.GetMesh().SetColor(chrono.ChColor(0.8, 0.7, 0.6)) 


rover = viper.Viper(system)  
driver = viper.ViperDCMotorControl()  
rover.SetDriver(driver)  






init_pos = chrono.ChVector3d(0, 0.2, 0.3) 
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
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, rover.GetChassis().GetPos().z)) 
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)





time_step = 1e-3


time = 0
while vis.Run():
    time += time_step  
    
    
    
    
    
    
    
    
    
    
    
    steering = 0.0  
    
    driver.SetSteering(steering)  

    
    

    rover.Update()  

    
    vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1)) 
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)