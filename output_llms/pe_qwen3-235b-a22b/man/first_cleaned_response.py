import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.path.expanduser("~"), "PyChrono_data"))
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        veh.RigidTerrain.PatchType_BOX, 
                        200, 100)  
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


vehicle = veh.MAN_10t(sys)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TMEASY)  
vehicle.SetTireStepSize(1e-3)    
vehicle.Initialize()


driver = veh.ChInteractiveDriverIRR(sys, vehicle, None, 20)
driver.SetSteeringDelta(0.05)    
driver.SetThrottleDelta(0.05)    
driver.SetBrakingDelta(0.05)     


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('MAN 10t Truck Simulation - TMEASY Tires on Rigid Terrain')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6), vehicle.GetChassis().GetPos())  
vis.AddTypicalLights(chrono.ChVector3d(30, 100, -50),  
                     chrono.ChColor(0.7, 0.7, 0.7),    
                     chrono.ChColor(1.0, 1.0, 1.0),    
                     chrono.ChColor(0.8, 0.8, 0.8))    


while vis.Run():
    time = sys.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    sys.DoStepDynamics(1e-3)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver.Advance(1e-3)