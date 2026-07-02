import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added as per instruction
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatIdentity()))
terrain.SetLength(20)
terrain.SetWidth(20)
terrain.SetMeshResolution(20)
terrain.SetSoilParameters(2e6, 0, 3e8, 1.1e4, 2e6, 4e4, 38)
terrain.Initialize()

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChCoordsysd(init_pos, init_rot))  # Corrected from ChFramed to ChCoordsysd

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
    
    # Set steering to 0.0 as per instruction
    driver.SetSteering(0.0)  # Set the steering for the rover

    rover.Update()  # Update the rover's state

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)