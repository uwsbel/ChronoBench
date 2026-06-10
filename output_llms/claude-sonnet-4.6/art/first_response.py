# =============================================================================
# PyChrono ARTcar Vehicle Simulation on Rigid Terrain
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# =============================================================================
# Simulation Parameters
# =============================================================================
# Simulation step size
step_size = 1e-3

# Simulation end time
t_end = 1000.0

# Render FPS
render_fps = 50
render_step_size = 1.0 / render_fps

# =============================================================================
# Vehicle Initial Conditions
# =============================================================================
# Initial vehicle location and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# =============================================================================
# Terrain Parameters
# =============================================================================
terrain_length = 200.0   # Terrain length (X direction)
terrain_width  = 100.0   # Terrain width  (Y direction)
terrain_height = 0.0     # Terrain height (Z position)

# =============================================================================
# Initialize the ARTcar Vehicle
# =============================================================================
# Create the ARTcar vehicle system
vehicle = veh.ARTcar()

# Set contact method
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)

# Set chassis collision type
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)

# Fix chassis to ground (disable for free motion)
vehicle.SetChassisFixed(False)

# Set initial position and orientation
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Set visualization types for vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# Initialize the vehicle
vehicle.Initialize()

# Get the vehicle system
sys = vehicle.GetSystem()

# Print vehicle info
print("Vehicle mass:           ", vehicle.GetVehicle().GetMass())

# =============================================================================
# Create the Rigid Terrain
# =============================================================================
terrain = veh.RigidTerrain(sys)

# Create a terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width
)

# Set terrain patch visual properties
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize the terrain
terrain.Initialize()

# =============================================================================
# Create the Interactive Driver System
# =============================================================================
# Create the Irrlicht visualization application first (needed for driver)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar - Rigid Terrain Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the interactive driver
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle inputs
steering_time = 1.0    # Time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0    # Time to go from 0 to +1
braking_time  = 0.3    # Time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Initialize the driver
driver.Initialize()

# =============================================================================
# Simulation Loop
# =============================================================================
print("=============================================================================")
print("ARTcar Simulation - Rigid Terrain")
print("=============================================================================")
print("Use arrow keys to control the vehicle:")
print("  UP    - Throttle")
print("  DOWN  - Brake")
print("  LEFT  - Steer Left")
print("  RIGHT - Steer Right")
print("=============================================================================")

# Simulation counters
step_number    = 0
render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation time
sim_time = 0.0

while vis.Run():
    sim_time = vehicle.GetSystem().GetChTime()

    # End simulation
    if sim_time >= t_end:
        break

    # Render scene at specified FPS
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (order matters!)
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step counter
    step_number += 1

    # Spin in place to maintain real-time factor of 1
    realtime_timer.Spin(step_size)

print("\nSimulation ended at t = ", sim_time, "s")