import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions
terrainHeight = 0
terrainLength = 50.0  # Single patch length
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50  # 50 FPS

# Create the Gator vehicle
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set vehicle visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create different terrain patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Flat terrain with tile4 texture
patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-75, 0, 0), chrono.QUNIT),
    50, terrainWidth)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Flat terrain with tile2 texture
patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 0, 0), chrono.QUNIT),
    50, terrainWidth)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

# Patch 3: Heightmap for gradability (slope)
patch3 = terrain.AddPatch(patch_mat,
    veh.GetDataFile("heightmaps/slope.png"),
    chrono.ChVector3d(25, 0, 0.5),  # Centered at z = (0 + 1.0)/2
    50, terrainWidth,
    0, 1.0)
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.4, 0.7, 0.4))

# Patch 4: Bump heightmap
patch4 = terrain.AddPatch(patch_mat,
    veh.GetDataFile("heightmaps/bump10.png"),
    chrono.ChVector3d(75, 0, 0.25),  # Centered at z = (0 + 0.5)/2
    50, terrainWidth,
    0, 0.5)
patch4.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator Vehicle with Multi-Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set driver input mapping
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render frames
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

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