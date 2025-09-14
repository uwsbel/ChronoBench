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

# Set visualization parameters before initialization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.Initialize()

# Create terrain with 4 different patches
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Flat terrain with tile texture
patch1 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(12.5, 0, 0), chrono.QUNIT), 
    25, 25)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: Bumpy terrain with concrete texture
patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(37.5, 0, 0), chrono.QUNIT),
    25, 25)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 50, 50)
patch2.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

# Add bump obstacle
bump = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(37.5, 0, 0.25), chrono.QUNIT),
    5, 5, 0.5)
bump.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

# Patch 3: Heightmap terrain with dirt texture
patch3 = terrain.AddHeightfield(patch_mat,
    veh.GetDataFile("terrain/heightmaps/bump.bmp"),
    chrono.ChVector3d(62.5, 0, 0),
    25, 25, 0, 3)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 50, 50)
patch3.SetColor(chrono.ChColor(0.6, 0.4, 0.3))

# Patch 4: Sloped terrain with grass texture
patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(87.5, 0, 0), chrono.QUNIT),
    25, 25)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 50, 50)
patch4.SetColor(chrono.ChColor(0.3, 0.6, 0.3))

terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

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