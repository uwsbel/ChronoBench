import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added vehicle module for deformable terrain
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain (replacing rigid ground)
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(veh.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.Initialize(20, 20, 0.1)  # Length = 20m, Width = 20m, Resolution = 0.1m

# Set soil parameters (Bekker model)
Kphi = 0.82e6  # Frictional modulus [Pa]
Kc = 0.14e4    # Cohesive modulus [Pa]
n = 1.0         # Sinkage exponent
coh = 3e4       # Cohesion [Pa]
mu = 0.3        # Friction coefficient
H = 0.2         # Sinkage coefficient [m]
m = 0.2         # Sinkage exponent
Kn = 4e6        # Normal stiffness [Pa/m]
Gn = 5e7        # Normal damping [Pa.s/m]
Kt = 2e4        # Tangential stiffness [Pa/m]
Gt = 5e4        # Tangential damping [Pa.s/m]

terrain.SetSoilParameters(Kphi, Kc, n, coh, mu, H, m, Kn, Gn, Kt, Gt)

# Configure terrain visualization
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
texture_file = chrono.GetChronoDataFile("textures/grass.jpg")
terrain.SetTexture(texture_file, 20, 20)

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
vis.SetWindowTitle('Viper rover - Deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    
    # Constant steering set to 0.0 (removed time-varying behavior)
    steering = 0.0
    driver.SetSteering(steering)  # Set the steering for the rover

    rover.Update()  # Update the rover's state

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)