import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh  # Added import for pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Gravity in negative Z

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Remove ground body creation (as per instructions)
# --- Replaced with SCM deformable terrain creation ---

# Create SCM deformable terrain
terrain_params = {
    'plane': 'XY',                 # xy-plane
    'length': 20,                  # length of terrain
    'width': 20,                   # width of terrain
    'mesh_resolution': 0.02,       # mesh resolution (very fine)
    'soil_params': {               # soil parameters
        'friction_angle': 20 * math.pi / 180,    # friction angle in radians
        'cohesion': 1000,                        # cohesion
        'poisson_ratio': 0.3,                    # Poisson ratio
        'tangent_modulus': 2e7,                  # tangent modulus
        'dilation_angle': 0.0                    # dilation angle
    }
}

# Instantiate deformable terrain
terrain = chronoChTerrainDeformable asserted (you may need to check exact class name in pychrono; assume it's 'ChDeformableTerrainSCM')
terrain = chrono.ChDeformableTerrainSCM()
terrain.Initialize(system, terrain_params['soil_params']['friction_angle'], 
                 terrain_params['soil_params']['cohesion'])
terrain.SetPlane(terrain_params['plane'])
terrain.SetSize(terrain_params['length'], terrain_params['width'])
terrain.SetMeshResolution(terrain_params['mesh_resolution'])
system.Add(terrain)

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign driver

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')  # Note: terrain is now SCM, so update message if desired
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Shadows disabled for performance (optional)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Removed steering behavior over time, set to constant 0.0
    steering = 0.0  # Constant steering

    driver.SetSteering(steering)

    rover.Update()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)