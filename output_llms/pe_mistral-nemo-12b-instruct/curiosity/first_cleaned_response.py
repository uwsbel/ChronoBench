import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as robosimian


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', '..', 'data'))


sys = chrono.ChSystemSMC()
sys.Set_Gravity(chrono.ChVector3d(0, 0, -9.81))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


car = veh.Curiosity()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionModel.CollisionType_MULTICONTACT)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
car.Initialize()


driver = robosimian.RS_Driver(0.1, 0.1, 0.1, True)
car.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()


vis.AddCamera(chrono.ChVector3d(0, 5, -10))


vis.AddTypicalLights()
vis.AddSkyBox()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


while vis.Run():
    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()

    
    vis.Render()

    
    vis.EndScene()

    
    driver.Update()