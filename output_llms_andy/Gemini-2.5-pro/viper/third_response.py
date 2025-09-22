import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh # Instruction 1: Import pychrono.vehicle as veh

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Instruction 2: Ground body creation replaced with SCM deformable terrain
# --- Removed original ground body creation code ---
# ground_mat = chrono.ChContactMaterialNSC()
# ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
# ground.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the ground slightly below the origin
# ground.SetFixed(True)  # Fix the ground in place
# ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
# system.Add(ground)
# --- End of removed code ---

# Create SCM deformable terrain
terrain = chrono.SCMDeformableTerrain(system)

# Set SCM terrain parameters
# Position the SCM terrain plane (reference surface at Z=0)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# Define soil parameters. These are illustrative and can be adjusted for specific soil types.
# Parameters: K_c (Bekker cohesion), K_phi (Bekker friction), n (Bekker exponent),
# C_coh (Mohr-Coulomb cohesion [Pa]), phi (Mohr-Coulomb friction angle [degrees]),
# K_s (Janosi shear deformation [m]), E_pl (Elastic modulus for plastic deformation [Pa]),
# nu_s (Poisson's ratio for soil)
terrain.SetSoilParameters(
    0.2e6,  # K_c Bekker cohesion multiplyer [N/m^(n+1)]
    0,      # K_phi Bekker friction multiplyer [N/m^(n+1)]
    1.1,    # n Bekker exponent [-]
    20e3,   # C_coh Mohr-Coulomb cohesion [Pa]
    30,     # phi Mohr-Coulomb friction angle [degrees]
    0.01,   # K_s Janosi shear parameter [m]
    2e8,    # E_pl Elastic stiffness for virgin loading [Pa] (Young's modulus for virgin loading)
    0.3     # nu_s Poisson ratio of soil
)

# Initialize SCM terrain dimensions and mesh resolution
terrain_length = 20.0  # meters
terrain_width = 20.0   # meters
mesh_resolution = 0.4  # meters (grid cell size)
terrain.Initialize(terrain_length, terrain_width, mesh_resolution)

# Set SCM terrain visualization properties
# PLOT_MESH: Use this to visualize the SCM terrain mesh.
# The second argument is an optional texture name (not used here).
# The third argument can be a Z-level for color modulation if applicable.
terrain.SetPlotType(chrono.SCMDeformableTerrain.PLOT_MESH, "SCM_Mesh", 0.0)
# Set a default color for the SCM terrain mesh for better visibility
terrain.GetMesh().SetColor(chrono.ChColor(0.8, 0.7, 0.6)) # Light brown/tan color

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
# Original init_pos = chrono.ChVector3d(0, 0.2, 0) was for ground top surface at Z=-0.5.
# New SCM terrain surface is at Z=0.
# Viper wheel radius is ~0.165m. Chassis CoG height should be > wheel radius.
# Setting chassis CoG height to 0.3m places wheels ~0.135m above SCM Z=0 surface initially.
init_pos = chrono.ChVector3d(0, 0.2, 0.3) # Adjusted Z for SCM terrain at Z=0
init_rot = chrono.ChQuaterniond(1, 0, 0, 0) # Default orientation (no rotation)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - SCM Deformable Terrain') # Updated window title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, rover.GetChassis().GetPos().z)) # Camera target Z near rover's height
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (can be performance-intensive but good for SCM visualization)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    
    # Instruction 3: Removed steering behavior over time and set steering to a constant value of 0.0.
    # --- Removed original time-dependent steering logic ---
    # steering = 0
    # max_steering = math.pi / 6
    # if 2 < time < 7:
    #     steering = max_steering * (time - 2) / 5
    # elif 7 < time < 12:
    #     steering = max_steering * (12 - time) / 5
    # --- End of removed code ---
    
    steering = 0.0  # Set steering to a constant value of 0.0
    
    driver.SetSteering(steering)  # Set the steering for the rover

    # Note: The rover will not move forward as no throttle/motor speed command is given.
    # To enable movement, one could use: driver.SetDriveSpeed(target_speed_mps)

    rover.Update()  # Update the rover's state

    # Render the scene
    vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1)) # Clear with a dark gray background
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

# Simulation ends when visualization window is closed
# PyChrono objects are typically deallocated automatically when they go out of scope.