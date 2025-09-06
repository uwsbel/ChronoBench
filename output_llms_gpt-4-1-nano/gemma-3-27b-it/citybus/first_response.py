import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("./chrono_data")  # Replace with your actual path

# ---------------------------------------------------------------------
#
#  CITYBUS Simulation
#
# ---------------------------------------------------------------------

# Create a Chrono system
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set gravity
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

# ---------------------------------------------------------------------
#
#  RIGID TERRAIN
#
# ---------------------------------------------------------------------

# Create the rigid terrain
terrain = veh.RigidTerrain(sys)
terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain/grass.png"))
terrain.SetTerrainSize(200, 200)
terrain.SetHeightDistribution(veh.RigidTerrain.HeightDistributionType.BUMP)
terrain.SetBumpHeight(0.1)
terrain.Initialize()


# ---------------------------------------------------------------------
#
#  VEHICLE
#
# ---------------------------------------------------------------------

# Create the CityBus vehicle
bus = veh.CityBus(sys)

# Set initial vehicle position and orientation
init_pos = chrono.ChVector3d(0, 0.5, 0)
init_rot = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))
bus.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))

# Set tire model
bus.SetTireModel(veh.CityBus.TireModelType.RADIAL)

# Set vehicle parameters (adjust as needed)
bus.SetEngineType(veh.CityBus.EngineType.ELECTRIC)
bus.SetTransmissionType(veh.CityBus.TransmissionType.AUTOMATIC)
bus.SetBrakeType(veh.CityBus.BrakeType.HYDRAULIC)
bus.SetSteeringType(veh.CityBus.SteeringType.POWER_STEERING)

# Initialize the vehicle
bus.Initialize()

# ---------------------------------------------------------------------
#
#  VISUALIZATION
#
# ---------------------------------------------------------------------

# Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(10, 5, -10))
vis.AddTypicalLights()
vis.AddSkyBox()

# Add a logo
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# ---------------------------------------------------------------------
#
#  DRIVER SYSTEM
#
# ---------------------------------------------------------------------

# Create a driver system for controlling the vehicle
driver = veh.CityBus.Driver(bus)

# ---------------------------------------------------------------------
#
#  SIMULATION LOOP
#
# ---------------------------------------------------------------------

# Simulation loop
time_step = 0.005
fps = 50
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the driver controls (example: constant throttle and steering)
    driver.SetThrottle(0.5)
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)

    # Advance the simulation
    sys.DoStepDynamics(time_step)
    
    # Adjust simulation speed to maintain desired FPS
    chrono.ChTimer::Sleep(1.0/fps)