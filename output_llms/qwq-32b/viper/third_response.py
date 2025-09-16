import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh  # Added import for vehicle module

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
plane_pos = chrono.ChVectorD(0, 0, -1)  # Position matching original ground's z-coordinate
plane_rot = chrono.ChQuaternionD(1, 0, 0, 0)
terrain_plane = chrono.ChCoordsysD(plane_pos, plane_rot)
dimensions = chrono.ChVectorD(20, 20, 1)  # Length, width, thickness
meshX = 50  # Mesh resolution along X
meshY = 50  # Mesh resolution along Y
shear_mod = 1e6  # Shear modulus (Pa)
bulk_mod = 3e6  # Bulk modulus (Pa)
density = 1500  # Density (kg/m³)
damping = 0.1  # Damping coefficient

terrain = veh.ChDeformableTerrainSCM()
terrain.Initialize(terrain_plane, system, dimensions, meshX, meshY, shear_mod, bulk_mod, density, damping)
system.Add(terrain)

# Create Viper rover
rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - SCM Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Simulation loop
time_step = 1e-3
time = 0

while vis.Run():
    time += time_step
    steering = 0.0  # Constant steering set to 0
    
    driver.SetSteering(steering)
    rover.Update()
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(time_step)