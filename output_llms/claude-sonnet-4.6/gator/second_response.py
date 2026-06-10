import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
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

# Point on chassis tracked by the camera
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

# -------------------------------------------------------
# Patch 1: Flat starting patch with tile texture
# -------------------------------------------------------
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)
patch1 = terrain.AddPatch(
    patch_mat1,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# -------------------------------------------------------
# Patch 2: Height map patch for gradability testing
# Placed adjacent to Patch 1 in the +X direction
# -------------------------------------------------------
patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.9)
patch_mat2.SetRestitution(0.01)
patch2 = terrain.AddPatch(
    patch_mat2,
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/test64.bmp"),
    terrainLength, terrainWidth,
    0,    # minimum height
    6.0   # maximum height (creates a noticeable grade)
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)
patch2.SetColor(chrono.ChColor(0.3, 0.6, 0.3))

# -------------------------------------------------------
# Patch 3: Flat patch with concrete texture
# Placed adjacent to Patch 1 in the +Y direction
# -------------------------------------------------------
patch_mat3 = chrono.ChContactMaterialNSC()
patch_mat3.SetFriction(0.8)
patch_mat3.SetRestitution(0.01)
patch3 = terrain.AddPatch(
    patch_mat3,
    chrono.ChCoordsysd(chrono.ChVector3d(0, terrainWidth, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

# -------------------------------------------------------
# Patch 4: Bump patch using a height map
# Placed diagonally (adjacent to both Patch 2 and Patch 3)
# The bump height map creates uneven terrain with bumps
# -------------------------------------------------------
patch_mat4 = chrono.ChContactMaterialNSC()
patch_mat4.SetFriction(0.9)
patch_mat4.SetRestitution(0.01)
patch4 = terrain.AddPatch(
    patch_mat4,
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength, terrainWidth, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    terrainLength, terrainWidth,
    0,    # minimum height
    2.0   # maximum height (creates noticeable bumps)
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 100, 100)
patch4.SetColor(chrono.ChColor(0.6, 0.4, 0.2))

terrain.Initialize()

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

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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