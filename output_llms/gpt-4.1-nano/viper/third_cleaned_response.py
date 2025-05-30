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





terrain_params = {
    'plane': 'XY',                 
    'length': 20,                  
    'width': 20,                   
    'mesh_resolution': 0.02,       
    'soil_params': {               
        'friction_angle': 20 * math.pi / 180,    
        'cohesion': 1000,                        
        'poisson_ratio': 0.3,                    
        'tangent_modulus': 2e7,                  
        'dilation_angle': 0.0                    
    }
}


terrain = chronoChTerrainDeformable asserted (you may need to check exact class name in pychrono; assume it's 'ChDeformableTerrainSCM')
terrain = chrono.ChDeformableTerrainSCM()
terrain.Initialize(system, terrain_params['soil_params']['friction_angle'], 
                 terrain_params['soil_params']['cohesion'])
terrain.SetPlane(terrain_params['plane'])
terrain.SetSize(terrain_params['length'], terrain_params['width'])
terrain.SetMeshResolution(terrain_params['mesh_resolution'])
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
vis.SetWindowTitle('Viper rover - Rigid terrain')  
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