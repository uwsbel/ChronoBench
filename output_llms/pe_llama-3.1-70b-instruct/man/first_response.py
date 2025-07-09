import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono import veh as veh
from pychrono import irr as irr
from pychrono.vehicle import ChPart, ChPartVisual, ChVisualization
from pychrono import vehicle as veh
from pychrono import ChUtils as utils
from pychrono import ChFunction as func

# Create the simulation system and visualization (rendering)
my_system = chrono.ChSystemNSC()
vis = irr.ChVisualSystemIrrlicht(my_system)

# Set up the path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the vehicle
car = veh.MAN_10t_Truck()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionType_AABBTREE)
car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
car.Initialize(my_system)

# Create the terrain
terrain = veh.RigidTerrain(car.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Add visualization assets
car.AddVisualization(ChVisualization.CH_VISUALIZATION_MESH)
car.SetChassisBodyVisualization(ChVisualization.CH_VISUALIZATION_MESH)
car.SetSuspensionVisualization(ChVisualization.CH_VISUALIZATION_NONE)
car.SetSteeringVisualization(ChVisualization.CH_VISUALIZATION_MESH)
car.SetWheelVisualization(ChVisualization.CH_VISUALIZATION_MESH)
car.SetTireVisualization(ChVisualization.CH_VISUALIZATION_MESH)

# Create the vehicle driver
driver = veh.ChIrrGuiDriver(car)
car.AddDriver(driver)

# Set up the camera
vis.SetTypicalCamera(chrono.ChVectorD(0, 1.5, -3), chrono.ChVectorD(0, 1, 0))

# Set up the skybox
vis.AddSkyBox()

# Set up the logo
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))

# Set up the lighting
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

# Remove the vehicle and terrain when the simulation is done
my_system.Remove(car)
my_system.Remove(terrain)