import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted to be on heightmap patch)
initLoc = chrono.ChVector3d(-25, -25, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # base terrain height
patch_size = 50.0      # size of each terrain patch

# Poon chassis tracked by the camera
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

# Set visualization modes BEFORE initializing
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.Initialize()

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with four different patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Heightmap terrain (lower-left quadrant)
heightmap = terrain.AddHeightfield(
    patch_mat, 
    veh.GetDataFile("heightmaps/bump64.bmp"),  # Height map image
    50, 50,  # X and Y size
    0, 0.5,  # Height range
    chrono.ChVector3d(-25, -25, 0),  # Center position
    chrono.QUNIT,
    0.1  # Vertical scale
)
heightmap.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 50, 50)
heightmap.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# Patch 2: Concrete with bump (lower-right quadrant)
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0), chrono.QUNIT),
    patch_size, patch_size, 0.1  # Main patch
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 50, 50)
patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

# Add bump to patch2
bump2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0.3), chrono.QUNIT),
    2, 2, 0.4  # Bump dimensions
)
bump2.SetTexture(veh.GetDataFile("terrain/textures/stone.jpg"), 2, 2)

# Patch 3: Dirt with bump (upper-left quadrant)
patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0), chrono.QUNIT),
    patch_size, patch_size, 0.1
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 50, 50)
patch3.SetColor(chrono.ChColor(0.6, 0.4, 0.3))

# Add bump to patch3
bump3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0.3), chrono.QUNIT),
    2, 2, 0.4
)
bump3.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), 2, 2)

# Patch 4: Tile with bump (upper-right quadrant)
patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0), chrono.QUNIT),
    patch_size, patch_size, 0.1
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
patch4.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Add bump to patch4
bump4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0.3), chrono.QUNIT),
    2, 2, 0.4
)
bump4.SetTexture(veh.GetDataFile("terrain/textures/bump.png"), 2, 2)

terrain.Initialize()

# Create the vehicle Irrlicht interface
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

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop setup
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)