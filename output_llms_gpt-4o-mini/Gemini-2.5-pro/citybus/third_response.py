import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # Not strictly necessary for this script but good practice for path handling

# Set the path to the Chrono data files (textures, CAD models, etc.)
# It's good practice to ensure CHRONO_DATA_DIR is set in your environment,
# or set it programmatically if GetChronoDataPath() doesn't find it.
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', chrono.GetChronoDataPath()))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))


# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH
# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY, PACEJKA, etc.)
# MODIFICATION 1: Change tire model to Pacejka
tire_model = veh.TireModelType_PACEJKA # Changed from TMEASY

# Rigid terrain parameters
terrainHeight = 0      # reference Z height of the terrain (actual surface may depend on patch thickness)
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
# CORRECTED COMMENT: "Poon chassis" to "Point on chassis"
trackPoint = chrono.ChVector3d(-15.0, 10.0, 5.8) # Relative to chassis REF frame

# Contact method
contact_method = chrono.ChContactMethod_NSC
# REMOVED: Unused variable contact_vis

# Simulation step sizes
# MODIFICATION 2: Reduce simulation step size
step_size = 5e-4       # Changed from 1e-3
tire_step_size = step_size # Tire step size is coupled to simulation step size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the CityBus vehicle, set parameters, and initialize
vehicle = veh.CityBus()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization types for vehicle components
vehicle.SetChassisVisualizationType(vis_type_mesh)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type_mesh)
vehicle.SetTireVisualizationType(vis_type_mesh) # Pacejka tires are often visualized as meshes

# Set the collision system type (e.g., BULLET)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
# The patch is a box with default thickness 0.1, centered at Z=0 (terrainHeight)
# So its top surface will be at Z = terrainHeight + 0.1/2 = 0.05
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)

# MODIFICATION 3: Change terrain texture to "dirt.jpg"
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200) # Changed from tile4.jpg
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) # Color is less relevant when texture is applied
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('City Bus Demo with Pacejka Tires')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 3.5) # trackPoint is relative to chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system (interactive)
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
# These values represent the time (in seconds) to reach maximum input.
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events (e.g., rendering)
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# REMOVED: Unused variable terrain_model

# Simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0) :
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
    
    # Increment step number
    step_number += 1
    
    # Spin in place for real time to catch up (optional)
    realtime_timer.Spin(step_size)