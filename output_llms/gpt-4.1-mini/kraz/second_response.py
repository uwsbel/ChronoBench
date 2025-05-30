import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)
# Adjusted initial rotation: keep no rotation (facing along x-axis)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera (updated as per instructions)
trackPoint = chrono.ChVector3d(3, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle, set parameters, and initialize

vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

# NOTE: The collision system type must be set before Initialize(), so the code likely sets this too late.
# Correction: set CollisionSystemType before vehicle.Initialize()

# So we move this line before Initialize()
# Correct approach: 
# vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) must precede Initialize()

# Correct order:
# -> Set collision system BEFORE Initialize()

# So rework above to:

vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # Move here
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)  # Changed chase camera distance per instructions
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())  # Changed from vehicle.GetTractor() to vehicle.GetVehicle()

# NOTE: The original code uses vehicle.GetTractor(), but veh.Kraz() derives from wheeled vehicles.
# Checking PyChrono vehicle definitions, Kraz inherits from ChWheeledVehicle, which has GetVehicle() method, not GetTractor().
# So replace calls to GetTractor() with GetVehicle()

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# ----------------------------------------------------------------
# Added: Double lane change maneuver sequence controlled by the driver
# We define a simple state machine based on time to steer:
#   0   - 2s: drive straight
#   2   - 6s: steer left (simulate lane change)
#   6   - 10s: steer right (cross back)
#   10  - 14s: steer left (second lane change)
#   14  - 18s: steer right (back to center)
#   >18s: drive straight
# We manually set steering input for driver system.
# ----------------------------------------------------------------

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update driver inputs manually for double lane change maneuver
    # Get current inputs
    inputs = driver.GetInputs()
    steering_input = 0.0

    # Define maneuver timing and steering amplitude
    # Max steering input ~0.5 (moderate turn)
    max_steering = 0.5

    # Lane change sequence:
    if 2.0 <= time < 6.0:
        steering_input = max_steering
    elif 6.0 <= time < 10.0:
        steering_input = -max_steering
    elif 10.0 <= time < 14.0:
        steering_input = max_steering
    elif 14.0 <= time < 18.0:
        steering_input = -max_steering
    else:
        steering_input = 0.0

    # Feed manual steering to driver
    # Override steering input in the driver system for the maneuver:
    # The driver returns a struct; override steering directly.
    # We keep throttle at 0.5 for moving forward, no braking.
    inputs.m_steering = steering_input
    inputs.m_throttle = 0.5
    inputs.m_braking = 0.0

    # Apply modified inputs to driver
    driver.SetInputs(inputs)

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs, terrain)
    vis.Synchronize(time, inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)