import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set the Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'chrono_data'))

# Create the physical system
sys = chrono.ChSystemSMC()

# Set gravity (default is already -9.81 m/s^2 in Y direction)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create rigid terrain with specified friction and restitution
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)  # Set friction coefficient
patch_mat.SetRestitution(0.1)  # Set restitution coefficient
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         200, 100)  # Terrain dimensions
terrain.Initialize()

# Initialize Kraz vehicle with initial conditions
kraz = veh.Kraz(sys)
init_pos = chrono.ChVector3d(0, 0.5, 0)  # Initial position (x, y, z)
init_rot = chrono.QUNIT  # No initial rotation
kraz.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
kraz.Initialize()

# Set up driver system
driver = veh.ChDriver(kraz.GetVehicle())  # Get the vehicle's driver interface
driver.Initialize()  # Initialize the driver system

# Configure visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Kraz Vehicle Simulation')
vis.AddCamera(chrono.ChVector3d(0, 3, -6))  # Camera position
vis.AddTypicalLights(chronoirr.IRRlichtDevice().getVideoDriver().getLightManager())
vis.AddSkyBox()
vis.Initialize()

# Simulation loop parameters
time_step = 0.01  # 100 Hz simulation
end_time = 10.0  # Run for 10 seconds

# Simulation loop
while vis.Run():
    current_time = sys.GetChTime()
    
    # End simulation after end_time
    if current_time > end_time:
        break
    
    # Update driver inputs (example: constant throttle and no steering)
    driver.SetThrottle(0.8)  # 80% throttle
    driver.SetSteering(0.0)  # No steering input
    
    # Synchronize vehicle and terrain
    kraz.Synchronize(current_time, driver.GetInputs())  # Update vehicle dynamics
    terrain.Synchronize(current_time)  # Update terrain state
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()