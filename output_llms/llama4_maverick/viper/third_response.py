import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-math.pi / 2)))
terrain.SetSoilParameters(
    0.2e6,      # Bekker Kphi
    -0.01e6,    # Bekker Kc
    1.0,        # Bekker n exponent
    0,          # Mohr cohesive limit (Pa)
    30 * math.pi / 180,  # Mohr friction limit (rad)
    0.01,       # Janosi shear coefficient (m)
    2e8,        # Elastic stiffness (Pa/m), before plastic yield
    3e4         # Damping (Pa s/m), must be zero if terrain is undamped x
)

# Set terrain dimensions
terrain_length = 20.0
terrain_width = 20.0
terrain_resolution = 0.02
terrain.SetMeshResolution(terrain_resolution)
terrain.Initialize(terrain_length, terrain_width, terrain_resolution)

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
vis.SetWindowTitle('Viper rover - SCM deformable terrain')
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
    time += time_step
    steering = 0.0  # Constant steering value
    driver.SetSteering(steering)
    rover.Update()
    terrain.Synchronize(time)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)