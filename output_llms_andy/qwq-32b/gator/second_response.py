import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain with four patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

nx = 10
ny = 10

# Patch 1: Flat (0 to 25m in X)
patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    25.0, terrainWidth)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Slope (25 to 50m in X)
slope_height_map = chrono.ChHeightMap()
slope_height_map.SetGridSize(nx, ny)
for i in range(nx):
    for j in range(ny):
        local_x = i * (25.0/(nx-1))
        local_y = j * (100.0/(ny-1)) - 50.0
        elevation = (local_x / 25.0) * 1.0  # 0 to 1m slope
        slope_height_map.SetHeight(i, j, elevation)
patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(25, 0, 0), chrono.QUNIT),
    25.0, terrainWidth, slope_height_map)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.6, 0.8, 0.6))

# Patch 3: Bump (50 to 75m in X)
bump_height_map = chrono.ChHeightMap()
bump_height_map.SetGridSize(nx, ny)
for i in range(nx):
    for j in range(ny):
        local_x = i * (25.0/(nx-1))
        local_y = j * (100.0/(ny-1)) - 50.0
        dx = local_x - 12.5
        dy = local_y
        dist_sq = dx*dx + dy*dy
        max_radius_sq = 25.0  # arbitrary value
        elevation = 0.5 * (1 - dist_sq/max_radius_sq) if dist_sq <= max_radius_sq else 0.0
        bump_height_map.SetHeight(i, j, elevation)
patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(50, 0, 0), chrono.QUNIT),
    25.0, terrainWidth, bump_height_map)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile3.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.8, 0.6, 0.6))

# Patch 4: Flat (75 to 100m in X)
patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(75, 0, 0), chrono.QUNIT),
    25.0, terrainWidth)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile1.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.8, 0.8))

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
vis.AttachVehicle(vehicle)  # Corrected from vehicle.GetVehicle()

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Simulation loop
print("VEHICLE MASS: ", vehicle.GetMass())

render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)