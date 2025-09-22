import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added import for vehicle module
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))  # Use ChVectorD for double precision
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
plane_length = 20.0
plane_width = 20.0
mesh_res_x = 0.2  # Mesh resolution in x-direction (adjust as needed)
mesh_res_z = 0.2  # Mesh resolution in z-direction
soil_density = 1500.0  # kg/m³
soil_cohesion = 10000.0  # Pa
soil_friction = 30.0 * math.pi / 180.0  # Friction angle in radians (30 degrees)
soil_adhesion = 0.0

terrain = veh.DeformableTerrainSCM(system)
terrain.SetPlaneLength(plane_length)
terrain.SetPlaneWidth(plane_width)
terrain.SetMeshResolutionX(mesh_res_x)
terrain.SetMeshResolutionZ(mesh_res_z)
terrain.SetSoilDensity(soil_density)
terrain.SetCohesion(soil_cohesion)
terrain.SetFrictionAngle(soil_friction)
terrain.SetAdhesion(soil_adhesion)
terrain.SetPos(chrono.ChVectorD(0, 0, -1))  # Position the terrain at ground level
terrain.Initialize()  # Initialize the terrain

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation (fixed ChFrameD typo)
init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Use ChFrameD for double precision

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

# Simulation loop
time_step = 1e-3
time = 0

while vis.Run():
    time += time_step
    steering = 0.0  # Constant steering value as per instructions
    
    driver.SetSteering(steering)  # Set steering to 0.0
    
    rover.Update()  # Update the rover's state
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(time_step)