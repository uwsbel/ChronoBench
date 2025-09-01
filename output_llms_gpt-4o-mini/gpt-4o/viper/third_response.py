import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added import statement for pychrono.vehicle
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.Initialize(20, 20, 0.1)  # Length, width, mesh resolution

# Set soil parameters
terrain.SetSoilParameters(2e6,  // Bekker Kphi
                          0,    // Bekker Kc
                          1.1,  // Bekker n exponent
                          0,    // Mohr cohesive limit (Pa)
                          30,   // Mohr friction limit (degrees)
                          0.01, // Janosi shear coefficient (m)
                          4e7,  // Elastic stiffness (Pa/m), before plastic yield
                          3e4)  // Damping (Pa s/m), proportional to negative vertical speed

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected method name to ChFrameD

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
    steering = 0.0  # Set steering to a constant value of 0.0
    
    driver.SetSteering(steering)  # Set the steering for the rover

    rover.Update()  # Update the rover's state

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)