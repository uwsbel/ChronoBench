import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as rob


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', '..', 'data'))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('BMW E90 Sedan Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


sys = chrono.ChSystemSMC()
sys.Set_Gravity(chrono.ChVector3d(0, -9.81, 0))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


car = veh.BMW_E90()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionModel.Type_MBT)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
car.Initialize()


tire_model = veh.TMEASY()
tire_model.SetTireType(veh.TMEASY.TireType_SEMIDIAGONAL)
tire_model.SetTireRadius(0.3)
tire_model.SetTireWidth(0.2)
tire_model.SetTireAspectRatio(0.5)
car.SetTireModel(tire_model)


driver = rob.RS_Driver()
driver.SetSteeringScaling(1.0)
driver.SetThrottleScaling(1.0)
driver.SetBrakeScaling(1.0)
car.SetDriver(driver)


sys.Add(car)


vis.AttachSystem(sys)
vis.AddCamera(chrono.ChVector3d(0, 5, -10), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    
    sys.DoStepDynamics(0.01)

    
    driver.SetSteering(math.sin(sys.GetChTime()) * 0.5)
    driver.SetThrottle(0.5)
    driver.SetBrake(0.0)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.IsUserRequestExit():
        break


vis.Shutdown()