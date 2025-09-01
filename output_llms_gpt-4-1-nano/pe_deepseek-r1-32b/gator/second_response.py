import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions
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

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define materials for different patches
materials = []
textures = [
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    veh.GetDataFile("terrain/textures/tile5.jpg"),
    veh.GetDataFile("terrain/textures/tile6.jpg"),
    veh.GetDataFile("terrain/textures/tile7.jpg")
]

# Create four different terrain patches
for i in range(4):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.9 - 0.2*i)  # Vary friction
    mat.SetRestitution(0.01 + 0.02*i)  # Vary restitution
    materials.append(mat)

# Patch 1: Flat terrain with texture
patch1 = terrain.AddPatch(materials[0],
    chrono.ChCoordsysd(chrono.ChVector3d(-50, -50, 0), chrono.QUNIT),
    50, 50)
patch1.SetTexture(textures[0], 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Flat terrain with different texture
patch2 = terrain.AddPatch(materials[1],
    chrono.ChCoordsysd(chrono.ChVector3d(50, -50, 0), chrono.QUNIT),
    50, 50)
patch2.SetTexture(textures[1], 200, 200)
patch2.SetColor(chrono.ChColor(0.7, 0.6, 0.4))

# Patch 3: Flat terrain with different texture
patch3 = terrain.AddPatch(materials[2],
    chrono.ChCoordsysd(chrono.ChVector3d(-50, 50, 0), chrono.QUNIT),
    50, 50)
patch3.SetTexture(textures[2], 200, 200)
patch3.SetColor(chrono.ChColor(0.6, 0.5, 0.3))

# Patch 4: Height map terrain for gradability testing
height_map = []
for x in range(50):
    row = []
    for y in range(50):
        # Create a sloped terrain with random height variations
        height = 0.5 * (x/50) + 0.2 * random.uniform(-1, 1)
        row.append(height)
    height_map.append(row)

patch4 = terrain.AddPatch(materials[3],
    chrono.ChCoordsysd(chrono.ChVector3d(50, 50, 0), chrono.QUNIT),
    50, 50)
patch4.SetHeightmap(height_map, 50, 50, 10, 10)
patch4.SetTexture(textures[3], 200, 200)
patch4.SetColor(chrono.ChColor(0.9, 0.8, 0.6))

# Add a bump to the terrain
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)

bump_patch = terrain.AddPatch(bump_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT),
    10, 10)  # Bump dimensions

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

# Initialize simulation frame counters
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