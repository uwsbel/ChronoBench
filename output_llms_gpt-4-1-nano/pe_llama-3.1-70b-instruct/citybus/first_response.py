import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono import veh as veh
from pychrono import irr as irr
from pychrono.vehicle import ChPart, ChPartVisual, ChVisualizationType
from pychrono.vehicle import ChVisualizationLevel
from pychrono import ChUtils as utils
from pychrono.core import ChCoordsys as coordsys

# Initialize the PyChrono environment
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system and visualization
system = chrono.ChSystemNSC()
vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("CityBus Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create the CityBus vehicle
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(chrono.ChCollisionType_AABB)
bus.SetInitPosition(coordsys.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
bus.Initialize(system)

# Create the rigid terrain
terrain = veh.RigidTerrain(bus.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, coordsys.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Set up the visualization for the vehicle
bus.AddVisualization(ChVisualizationType.PRIMITIVES)
bus.SetVisualizationLevel(ChVisualizationLevel.DETAILED)

# Set up the camera to follow the vehicle
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.SetCameraFollow(bus.GetChassisBody())

# Create the driver system
driver = veh.ChIrrGuiDriver()
driver.SetVehicle(bus)
driver.Initialize()

# Simulation loop
while vis.Run():
    bus.Synchronize(time_step=0.02)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

# Finalize the simulation
bus.Finalize()