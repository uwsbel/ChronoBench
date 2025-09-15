import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added vehicle import
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(veh.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.Initialize(20, 20, 0.2)  # Length_x=20, Length_y=20, mesh resolution=0.2
terrain.SetSoilParameters(3e5,   # Bekker Kphi
                          3e3,   # Bekker Kc
                          1.1,   # Bekker n exponent
                          5e4,   # Mohr cohesive limit (Pa)
                          30,    # Mohr friction angle (degrees)
                          1e3,   # Janosi shear parameter (m)
                          4e4,   # Elastic stiffness (Pa/m)
                          3e4)   # Damping (Pa s/m)
system.Add(terrain)

# Create Viper rover
rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation (fixed ChFrameD typo)
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected to ChFrameD

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
    
    # Constant zero steering
    steering = 0.0
    driver.SetSteering(steering)

    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation
    system.DoStepDynamics(time_step)