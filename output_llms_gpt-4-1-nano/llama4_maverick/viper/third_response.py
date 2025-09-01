import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added import statement
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1), chrono.QUNIT))  # Position the plane
terrain.Initialize(20, 20, 0.04)  # Length, width, mesh resolution

# Soil parameters
soil_params = veh.SCMDeformableTerrain.SoilParameters()
soil_params.m_Bekker_Kphi = 0.68e6  # Kphi (Frictional modulus)
soil_params.m_Bekker_Kc = 1.37e4   # Kc (Cohesive modulus)
soil_params.m_Bekker_n = 0.85      # n (Exponent)
soil_params.m_Mohr_cohesion = 1.3e3  # c (Cohesion)
soil_params.m_Mohr_friction = 30   # phi (Friction angle)
soil_params.m_Janosi_shear = 0.01  # Shear displacement
soil_params.m_elastic_K = 4e7      # Elastic stiffness
soil_params.m_damping_R = 3e4      # Damping

terrain.SetSoilParameters(soil_params)

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - SCM deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    steering = 0.0  # Set steering to a constant value
    
    driver.SetSteering(steering)  # Set the steering for the rover

    rover.Update()  # Update the rover's state
    terrain.Synchronize(time)  # Synchronize terrain

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)