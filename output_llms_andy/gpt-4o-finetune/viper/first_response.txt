import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr

# =============================================================================
# Initialize the PyChrono environment and core components
# =============================================================================
# Set the data directory for Chrono assets
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
# Define output directory for simulation results
out_dir = chrono.GetChronoOutputPath() + "VIPER"

# Create a Chrono physical system
system = chrono.ChSystemNSC()
# Set the collision detection method to non-smooth contacts (NSC)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSCH)
# Define the gravitational acceleration vector (units: m/s^2)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Gravity along negative Z-axis
# Specify the model for contact force calculations
ch_contact_model = chrono.ChContactMethodNSC
system.SetContactForceModel(ch_contact_model)

# Create a shared contact material for the simulation
inerter_mat = chrono.ChContactMaterialNSC()
# Set the friction coefficient for the material
inerter_mat.SetFriction(0.9)
# Define the restitution coefficient (elasticity) for collisions
inerter_mat.SetRestitution(0.01)

# Add object to the system
# Create the ground body as a reference point for the rover
ground = chrono.ChBody()
system.Add(ground)
ground.SetFixed(True)  # Fix the ground in space

# Create a terrain plane for the rover to interact with
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)  # Set friction for the terrain
terrain = chrono.ChContactSurfaceNSC()
terrain.SetMaterialSurface(patch_mat)
terrain.AddTri(chrono.ChVector3d(-100, -100, 0), chrono.ChVector3d(-100, 100, 0), chrono.ChVector3d(100, 100, 0))
terrain.AddTri(chrono.ChVector3d(-100, -100, 0), chrono.ChVector3d(100, 100, 0), chrono.ChVector3d(100, -100, 0))
terrain.GetMesh().SetWireframe(True)
ground.GetAssets().Add(terrain)

# =============================================================================
# Add the Viper rover and its driver
# =============================================================================
# Create and initialize the Viper rover
rover = viper.Viper(system)
rover.Initialize(chrono.ChFrame3d(chrono.ChVector3d(0, 0, 0.3), chrono.ChQuaterniond(1, 0, 0, 0)))

# Create the rover driver system
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# =============================================================================
# Set up the Irrlicht visualization
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 30, 512)

# =============================================================================
# Define simulation parameters
# =============================================================================
# Set the time step for the simulation (units: seconds)
time_step = 1e-3
# Initialize simulation time and frame counters
time = 0
time_end = 20  # Set the desired duration of the simulation (in seconds)
# Define the interval for rendering frames (in seconds)
render_step_size = 1.0 / 50  # Typically set for 50 frames per second
# Calculate the number of simulation steps between rendering frames
render_steps = math.ceil(render_step_size / time_step)
# Initialize counters for simulation steps and rendering frames
step_number = 0
render_frame = 0
tire_force = chrono.ChVector3d()

# Set the rover driver to active mode
driver.SetDrivingMode(viper.ViperDCMotorControl.Drivemode_ACTIVE)

# Simulation loop
while vis.Run():
    # Render the scene at specified intervals
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    # Gradually change the rover's steering after 1 second into the simulation
    if 1 < time < 4:
        driver.SetSteering((time - 1) / 3)
    # Update the rover's state based on driver inputs
    rover.Update()
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)
    # Increment the simulation time counter
    time += time_step
    step_number += 1