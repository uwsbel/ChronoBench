import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", 'data'))


sys = chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ARTcar Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


contact_method = chrono.ChContactMethod_SMC
chassis_collision_type = veh.ChassisCollisionType_BOX
initLoc = chrono.ChVector3d(0, 0.5, 0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


car = veh.ARTcar()
car.SetContactMethod(contact_method)
car.SetChassisCollisionType(chassis_collision_type)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.Initialize(sys)  


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
terrain.Initialize()


driver = veh.ChInteractiveDriver(car)
driver.SetSteeringDelta(0.05)  
driver.SetThrottleDelta(0.1)   
driver.SetBrakingDelta(0.1)    
driver.Initialize()


time_step = 0.02  


while vis.Run():
    time = sys.GetChTime()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    car.Synchronize(time, driver.GetInputs(), terrain)
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()