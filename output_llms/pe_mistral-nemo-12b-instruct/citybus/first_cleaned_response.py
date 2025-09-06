import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA'], '..'))


sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))


car = veh.CityBus()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionModel.NarrowPhase)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.Initialize()


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


camera = vis.GetCamera(0)
camera.SetLookAt(chrono.ChVector3d(0, 3, 6), chrono.ChVector3d(0, 0, 0))


driver = veh.CityBusDriver(car)
driver.SetSteeringFunction(chrono.ChFunction_Const(0))
driver.SetThrottleFunction(chrono.ChFunction_Const(0))
driver.SetBrakeFunction(chrono.ChFunction_Const(0))


while vis.Run():
    
    driver.Update(0.01)
    car.Update(0.01, False)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    chrono.ChTime.GetWallClock().Sleep(0.02)