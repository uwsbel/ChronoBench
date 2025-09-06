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
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with 4 different patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Texture paths
texture_paths = [
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    veh.GetDataFile("terrain/textures/tile5.jpg"),
    veh.GetDataFile("terrain/textures/tile6.jpg"),
    veh.GetDataFile("terrain/textures/tile7.jpg")
]

# Create 4 terrain patches with different textures and heights
patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength/2, terrainWidth/2)
patch1.SetTexture(texture_paths[0], 100, 100)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength/2, 0, 0), chrono.QUNIT),
    terrainLength/2, terrainWidth/2)
patch2.SetTexture(texture_paths[1], 100, 100)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))

# Create a height map for the third patch (for gradability testing)
height_map = veh.RigidTerrain.HeightMap()
height_map.SetSize(100, 100, 0.5, 0.5)  # 100x100 grid, 0.5m spacing
height_map.SetHeightRange(-0.2, 0.5)    # Height range from -0.2m to 0.5m

# Create a simple ramp in the height map
for i in range(100):
    for j in range(100):
        # Create a ramp that goes from 0 to 0.5m height
        height = 0.5 * (i / 99.0)
        height_map.SetHeight(i, j, height)

patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, terrainWidth/2, 0), chrono.QUNIT),
    terrainLength/2, terrainWidth/2)
patch3.SetTexture(texture_paths[2], 100, 100)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))
patch3.SetHeightMap(height_map)

# Create a patch with a bump
patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(terrainLength/2, terrainWidth/2, 0), chrono.QUNIT),
    terrainLength/2, terrainWidth/2)
patch4.SetTexture(texture_paths[3], 100, 100)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.2))

# Create a bump in the center of the fourth patch
bump_height = 0.3
bump_radius = 2.0
for i in range(50):
    for j in range(50):
        x = (i - 25) * 0.2  # Center at (0,0) with spacing
        y = (j - 25) * 0.2
        dist = math.sqrt(x*x + y*y)
        if dist < bump_radius:
            height = bump_height * (1 - dist/bump_radius)
            terrain.AddPatch(patch_mat,
                chrono.ChCoordsysd(chrono.ChVector3d(terrainLength/2 + x, terrainWidth/2 + y, height), chrono.QUNIT),
                0.2, 0.2).SetTexture(texture_paths[3], 1, 1)

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