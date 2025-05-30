import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set paths to Chrono data files
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH
# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction
# Removed unused variable: terrain_model = veh.RigidTerrain.BOX

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(-15.0, 10.0, 5.8) # Comment might be for a different vehicle

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Unused in this script, but not an error

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the citybus vehicle, set parameters, and initialize
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh)

# Set the collision system type for the entire system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), # Use terrainHeight here
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo with Data Driver')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 3.5) # Camera parameters
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --- Driver System Modification: Start ---
# Create the data-driven driver system
driver_data = veh.vector_Entry()
# Add driver input entries: (time, steering, throttle, braking)
# Note: steering is in range [-1,1], throttle [0,1], braking [0,1]
driver_data.push_back(veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0))  # Start
driver_data.push_back(veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0))  # Accelerate
driver_data.push_back(veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0))  # Accelerate and turn

# Create the ChDataDriver and provide the data and vehicle
# The ChDataDriver will interpolate inputs between the given time points
# and hold the last input values if simulation time exceeds the last data point time.
driver = veh.ChDataDriver(vehicle.GetVehicle(), driver_data)

# No explicit driver.Initialize() is needed for ChDataDriver when data is passed via constructor,
# as its constructor calls its own Initialize method.

# --- Driver System Modification: End ---

# --- Removed Time Response Settings for Interactive Driver ---
# The following section was for ChInteractiveDriverIRR and is no longer applicable:
# steering_time = 1.0
# throttle_time = 1.0
# braking_time = 0.3
# driver.SetSteeringDelta(render_step_size / steering_time)
# driver.SetThrottleDelta(render_step_size / throttle_time)
# driver.SetBrakingDelta(render_step_size / braking_time)
# --- Removed Time Response Settings: End ---

# Output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Cleanup (optional, as Python handles it, but good practice in some contexts)
# No explicit cleanup needed for this script structure.