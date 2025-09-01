import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


mysystem = chrono.ChSystemSMC()


m113 = veh.M113(mysystem)
m113.SetChassisCollisionType(chrono.ChCollisionType.BULLET)
m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_PI / 4)))
mysystem.Add(m113.GetChassisBody())


terrain = veh.RigidTerrain(mysystem)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChIrrVehicleDriver()
driver.SetVehicle(m113)
driver.SetSteeringDelta(0.4)
driver.SetSteeringRatio(15)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("M113 Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 6))
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChColor(1.2, 1.2, 1.2), 10, 90, 90, 120, 0.5)
vis.AddTypicalLigths()
vis.SetShowColliderSize(True)
vis.SetShowCollisionModels(True)


vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 6))
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChColor(1.2, 1.2, 1.2), 10, 90, 90, 120, 0.5)
vis.AddTypicalLigths()
vis.SetShowColliderSize(True)
vis.SetShowCollisionModels(True)


while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()