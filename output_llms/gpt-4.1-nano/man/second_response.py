import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Initial vehicle location and orientation ---
# Changed from (0, 0, 0.5) to (-20, 0, 1.5)
initLoc = chrono.ChVector3d(-20, 0, 1.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions and height
# Changed terrain to a terrain of rigid hills with height map (see below)
terrainHeight = 0      # base height (not used if height map is provided)
terrainLength = 100.0
terrainWidth = 100.0

# Track point for chase camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size (FPS=50)
render_step_size = 1.0 / 50

# --- Create vehicle ---
vehicle = veh.MAN_5t()  # Changed from veh.MAN_10t to veh.MAN_5t
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Create terrain with height map (rugged terrain of hills) ---
# Note: For height map terrain, assuming using RigidTerrain of type HEIGHTMAP or similar.
# Since the original code was for a patch, now modifying to height map terrain.
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Create height map terrain
# For height map, use CHLoadFile() method to load a heightmap image or data file
# Assuming a height map image file named "heightmap.png" located in data directory
heightmap_file = chrono.GetChronoDataFile("terrain/hills_heightmap.png")
terrain.Initialize()  # Initialize terrain before adding height map
# Specify the height map parameters: size and scale
terrain.AddHeightMap(
    heightmap_file,
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,  # size in X
    terrainWidth,   # size in Y
    0,              # height map scale (assuming default)
    0,              # vertical scale (assuming default)
    True            # whether to interpolate
)

# Set texture to "grass.jpg"
# Assuming the texture file exists at the specified location
patch = terrain.GetPatch(0)  # Access the first patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# --- Visualization ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 5t Terrain of Hills Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --- Driver system ---
driver = veh.ChInteractiveDriverIRR(vis)
# Set response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation parameters
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# --- Simulation loop ---
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    # Render scene at specified interval
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step count
    step_number += 1

    # Keep real-time pace
    realtime_timer.Spin(step_size)