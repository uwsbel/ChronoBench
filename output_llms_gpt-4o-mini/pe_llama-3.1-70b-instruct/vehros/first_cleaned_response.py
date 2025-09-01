import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as robosimian


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


my_system = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht(my_system)


veh_params = veh.HMMWV()
veh_params.SetContactMethod(chrono.ChContactMethod_NSC)
veh_params.SetEngineType(chrono.EngineType_IDEAL)
veh_params.SetTireModel(chrono.TireModel_RIGID)
veh_params.SetChassisCollisionType(chrono.ChassisCollisionType_MESH)
veh_params.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
veh_params.Initialize(my_system)


terrain = veh.RigidTerrain(my_system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChIrrVehicleDriver(veh_params.GetChassisBody())
driver.SetSteeringDelta(0.04)
driver.SetSteeringRatio(15)
driver.SetMaxSpeed(40)








time_step = 0.01
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 3, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    my_system.DoStepDynamics(time_step)
    driver.Synchronize(time_step)
    terrain.Synchronize(time_step)
    veh_params.Synchronize(time_step)
    vis.Synchronize(time_step)

    
    

    chrono.ChUtils::Sleep(0.01)