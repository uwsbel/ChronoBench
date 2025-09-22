import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
from pychrono import ChVectorD, ChQuaternionD
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Simulation end time
tend = 30

# Point on chassis tracked by the camera
camera_target = ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems and simulation
# =============================================================================

# Create the vehicle system
vehicle = veh.WheelVehicle(veh.GetDataFile('vehicle/man_10t/vehicle.json'))

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(ChVectorD(0, 0, 0), ChVectorD(0, 0, 1))

# Create the driver system
driver = veh.ChDriver()
driver.Initialize(vehicle)

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChCamera.Chase)
vis.SetCameraTrackingPoint(camera_target)
vis.GetCamera().SetAngle(chrono.ChPi / 6)
vis.GetCamera().SetPosition(chrono.ChVectorD(0, -6, 1.75))
vis.GetCamera().SetZoom(15)

# Create the vehicle visualization
veh_vis = veh.ChWheelVehicleVisualSystemIrrlicht()
veh_vis.SetVehicle(vehicle)
veh_vis.Initialize()
veh_vis.SetChaseCamera(camera_target, 6.0, 0.5)

# Create the terrain visualization
terrain_vis = veh.ChTerrainVisualSystemIrrlicht(terrain)
terrain_vis.Initialize()

# =============================================================================
# Initialize the vehicle
# =============================================================================

# Set the initial vehicle position and orientation
vehicle.Initialize(ChVectorD(0, 0, 0.5), ChQuaternionD(1, 0, 0, 0), 0)

# =============================================================================
# Main simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
render_steps = int(1 / (step_size * 60))

# Initialize simulation frame counter
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Exit simulation loop when end time is reached
    if time >= tend:
        break

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        veh_vis.Render()
        terrain_vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    # Increment frame number
    step_number += 1

# =============================================================================
# Clean up
# =============================================================================

# Remove visualization assets
vis.RemoveAllAssets()