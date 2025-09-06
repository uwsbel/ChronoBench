import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
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

# Terrain parameters
terrainHeight = 0.5      # Increased from 0 to create physical terrain
terrainLength = 100.0    # Size in X direction
terrainWidth = 100.0     # Size in Y direction

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

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

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with multiple patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Grass texture (flat)
patch1 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-37.5, 0, terrainHeight/2), chrono.QUNIT), 
    25, 100, terrainHeight)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.0, 1.0, 0.0))

# Patch 2: Sand texture (flat)
patch2 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-12.5, 0, terrainHeight/2), chrono.QUNIT), 
    25, 100, terrainHeight)
patch2.SetTexture(veh.GetDataFile("terrain/textures/sand.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(1.0, 1.0, 0.0))

# Patch 3: Sloped terrain (10 degrees) for gradability testing
slope_angle = 0.1745  # 10 degrees in radians
patch3 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(12.5, 0, terrainHeight/2), chrono.QuatFromAngleX(slope_angle)), 
    25, 100, terrainHeight)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.6, 0.3, 0.0))

# Patch 4: Concrete texture (flat)
patch4 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(37.5, 0, terrainHeight/2), chrono.QUNIT), 
    25, 100, terrainHeight)
patch4.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

# Add bump on patch4
bumpHeight = 0.3
bump_patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(37.5, 0, terrainHeight + bumpHeight/2), chrono.QUNIT), 
    5, 5, bumpHeight)
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/checker.jpg"), 50, 50)
bump_patch.SetColor(chrono.ChColor(1.0, 0.0, 0.0))

terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle - Multi-Patch Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set input response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop parameters
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Simulation loop
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

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Maintain real-time execution
    realtime_timer.Spin(step_size)