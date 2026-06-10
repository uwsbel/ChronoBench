import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -------------------------------------------------------------------
# 1. Initialize the PyChrono environment and set data paths
# -------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------------------------------------
# Simulation parameters
# -------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (MESH, PRIMITIVES, NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, NONE)
chassis_collision_type = veh.CollisionType_NONE

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Rigid terrain parameters
terrainHeight = 0.0
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# -------------------------------------------------------------------
# 2. Create the Kraz vehicle, set parameters, and initialize
# -------------------------------------------------------------------
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))

# Initialize the vehicle
vehicle.Initialize()

# Set visualization types for the vehicle subsystems
vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# -------------------------------------------------------------------
# Create the rigid terrain with defined friction and restitution
# -------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Create the contact material with friction and restitution
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Add a flat terrain patch
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

# Initialize the terrain
terrain.Initialize()

# -------------------------------------------------------------------
# 3. Create the Irrlicht visualization system with camera & lighting
# -------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Vehicle Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -------------------------------------------------------------------
# Create the interactive driver system for vehicle control
# -------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering and throttle keyboard inputs
steering_time = 1.0   # time to go from 0 to +1 (or 0 to -1)
throttle_time = 1.0   # time to go from 0 to +1
braking_time = 0.3    # time to go from 0 to +1

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Initialize the driver
driver.Initialize()

# -------------------------------------------------------------------
# Simulation loop setup
# -------------------------------------------------------------------
# Number of simulation steps between two render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
step_number = 0
render_frame = 0

# Create a real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

# -------------------------------------------------------------------
# Main simulation loop: synchronize and advance all systems
# -------------------------------------------------------------------
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at the prescribed render rate
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get current driver inputs
    driver_inputs = driver.GetInputs()

    # --- Synchronize each subsystem with current time/inputs ---
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # --- Advance the state of each subsystem by one timestep ---
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step counter
    step_number += 1

    # Spin the real-time timer to maintain real-time execution
    realtime_timer.Spin(step_size)