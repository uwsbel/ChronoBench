import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system and set the solver type
mysystem = chrono.ChSystemSMC()

# Create the vehicle, set its position, and add it to the system
m113 = veh.M113(mysystem)
m113.SetChassisCollisionType(chrono.ChCollisionType.BULLET)
m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_PI / 4)))
mysystem.Add(m113.GetChassisBody())

# Create the terrain, set its properties, and add it to the system
terrain = veh.RigidTerrain(mysystem)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Create the driver system
driver = veh.ChIrrVehicleDriver()
driver.SetVehicle(m113)
driver.SetSteeringDelta(0.4)
driver.SetSteeringRatio(15)

# Create the Irrlicht visualization
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

# Initialize the visualization system
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 6))
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChColor(1.2, 1.2, 1.2), 10, 90, 90, 120, 0.5)
vis.AddTypicalLigths()
vis.SetShowColliderSize(True)
vis.SetShowCollisionModels(True)

# Simulation loop
while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()