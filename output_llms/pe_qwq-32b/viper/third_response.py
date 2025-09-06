import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# Create Chrono system using SMC contact method
system = chrono.ChSystemSMC()
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create deformable terrain using vehicle module
terrain = veh.DeformableTerrain(system)
soil = veh.SoilModel()
soil.SetYoungModulus(1e7)  # Set soil Young's modulus
soil.SetFrictionCoefficient(0.5)  # Set soil friction coefficient
position = chrono.ChVectorD(0, 0, -1)  # Position of terrain base
frame = chrono.ChFrameD(position)  # Create frame for terrain placement
patch_length = 20.0
patch_width = 20.0
mesh_resolution = 0.2  # Mesh spacing in meters
patch = terrain.AddPatch(soil, frame, patch_length, patch_width, mesh_resolution)
terrain.Initialize()

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation using double precision vectors
init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

# Simulation parameters
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step
    
    # Set constant steering to 0.0 (removed time-based steering)
    driver.SetSteering(0.0)
    
    rover.Update()
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(time_step)