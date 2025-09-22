import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh                  # Added import as per instruction
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Gravity in negative Z
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain (replace rigid ground)
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  # horizontal plane at z=0
terrain.SetLength(20)
terrain.SetWidth(20)
terrain.SetMeshResolution(0.2)  # mesh size for terrain discretization

# Set soil parameters (example values taken from typical SCM soil params)
# For reference: https://api.projectchrono.org/classchrono_1_1vehicle_1_1_s_c_m_deformable_terrain.html
terrain.SetSoilParameters(2e6,    # Bekker Kphi [Pa/m^(n+1)]
                          0,      # Bekker Kc [Pa]
                          1.1,    # Bekker n (dimensionless)
                          0,      # Mohr cohesive limit (Pa)
                          30,     # Mohr friction limit (degrees)
                          2e7,    # Janosi shear coefficient [Pa/m]
                          0.01,   # elastic stiffness (vertical) [m]
                          3e4)    # damping (vertical) [N·s/m]
terrain.Initialize()

# Create Viper rover
rover = viper.Viper(system)  # Instantiate Viper rover
driver = viper.ViperDCMotorControl()  # Create motor driver for rover
rover.SetDriver(driver)  # Assign driver

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0.3)  # slightly above terrain, prevent initial penetration
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected to ChFrameD

# Create visualization system
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

# Enable shadows if desired - commented out to improve performance
# vis.EnableShadows()

# Simulation time step
time_step = 1e-3

# Simulation loop
while vis.Run():
    # Steering removed per instructions: always 0.0
    driver.SetSteering(0.0)

    rover.Update()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)