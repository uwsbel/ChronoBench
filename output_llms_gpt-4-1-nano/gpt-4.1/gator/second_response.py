import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataFile(""))  # Ensures data path is set
veh.SetDataPath(chrono.GetChronoDataFile("vehicle/"))

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 40.0   # size in X direction for each patch
terrainWidth = 40.0    # size in Y direction for each patch

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Flat, texture 1
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)
patch1 = terrain.AddPatch(
    patch_mat1,
    chrono.ChCoordsysd(chrono.ChVector3d(-terrainLength, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Flat, texture 2
patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.9)
patch_mat2.SetRestitution(0.01)
patch2 = terrain.AddPatch(
    patch_mat2,
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# Patch 3: Flat, texture 3
patch_mat3 = chrono.ChContactMaterialNSC()
patch_mat3.SetFriction(0.9)
patch_mat3.SetRestitution(0.01)
patch3 = terrain.AddPatch(
    patch_mat3,
    chrono.ChCoordsysd(chrono.ChVector3d(0, terrainWidth, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

# Patch 4: Height map, texture 4 (for gradability)
patch_mat4 = chrono.ChContactMaterialNSC()
patch_mat4.SetFriction(0.9)
patch_mat4.SetRestitution(0.01)
heightmap_file = veh.GetDataFile("terrain/height_maps/heightmap_bowl.bmp")
hMin = 0.0
hMax = 2.0
mesh_resolution = 0.2
patch4 = terrain.AddPatch(
    patch_mat4,
    heightmap_file,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -terrainWidth, 0), chrono.QUNIT),
    terrainLength, terrainWidth,
    hMin, hMax,
    mesh_resolution
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.6, 0.5, 0.3))

terrain.Initialize()

# Add bumps to each patch (as small boxes)
system = vehicle.GetSystem()
bump_size = chrono.ChVector3d(1.0, 3.0, 0.3)  # x, y, z
bump_height = bump_size.z / 2.0

# Bump on Patch 1
bump1 = chrono.ChBodyEasyBox(bump_size.x, bump_size.y, bump_size.z, 1000, True, True, patch_mat1)
bump1.SetPos(chrono.ChVector3d(-terrainLength, 0, bump_height))
bump1.SetBodyFixed(True)
system.Add(bump1)

# Bump on Patch 2
bump2 = chrono.ChBodyEasyBox(bump_size.x, bump_size.y, bump_size.z, 1000, True, True, patch_mat2)
bump2.SetPos(chrono.ChVector3d(terrainLength, 0, bump_height))
bump2.SetBodyFixed(True)
system.Add(bump2)

# Bump on Patch 3
bump3 = chrono.ChBodyEasyBox(bump_size.x, bump_size.y, bump_size.z, 1000, True, True, patch_mat3)
bump3.SetPos(chrono.ChVector3d(0, terrainWidth, bump_height))
bump3.SetBodyFixed(True)
system.Add(bump3)

# Bump on Patch 4 (height map patch)
bump4 = chrono.ChBodyEasyBox(bump_size.x, bump_size.y, bump_size.z, 1000, True, True, patch_mat4)
bump4.SetPos(chrono.ChVector3d(0, -terrainWidth, hMax + bump_height))
bump4.SetBodyFixed(True)
system.Add(bump4)

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

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

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
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