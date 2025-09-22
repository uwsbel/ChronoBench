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
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 200.0  # increased to 200.0 to accommodate circular path
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create circular path
path_radius = 20.0
path = chrono.ChLineCircularArc()
path.Set_ang_start(0)
path.Set_ang_end(2 * chrono.CH_C_PI)  # Full circle
path.Set_center(chrono.ChVectorD(0, 0, terrainHeight))
path.Set_plane_dir(chrono.ChVectorD(0, 0, 1))  # XY plane
path.Set_plane_uv(chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))
path.Set_radius(path_radius)

# Create path follower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  # Constant throttle
driver.SetSteeringPID(2.0, 0.0, 0.1)  # PID gains for steering

# Add visualization spheres for target and sentinel points
target_sphere = chrono.ChSphereShape()
target_sphere.GetSphereGeometry().rad = 0.3
target_sphere.SetColor(chrono.ChColor(1, 0, 0))  # Red for target
target_sphere.SetPos(chrono.ChVectorD(0, 0, terrainHeight))
vehicle.GetSystem().AddVisualizationShape(target_sphere)

sentinel_sphere = chrono.ChSphereShape()
sentinel_sphere.GetSphereGeometry().rad = 0.3
sentinel_sphere.SetColor(chrono.ChColor(0, 1, 0))  # Green for sentinel
sentinel_sphere.SetPos(chrono.ChVectorD(0, 0, terrainHeight))
vehicle.GetSystem().AddVisualizationShape(sentinel_sphere)

driver.Initialize()

# Simulation loop variables
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

    # Update driver and retrieve inputs
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    # Update visualization spheres
    target_point = driver.GetTargetPoint()
    sentinel_point = driver.GetSentinelPoint()
    target_sphere.SetPos(target_point)
    sentinel_sphere.SetPos(sentinel_point)

    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

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
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 200.0  # increased to 200.0 to accommodate circular path
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create circular path
path_radius = 20.0
path = chrono.ChLineCircularArc()
path.Set_ang_start(0)
path.Set_ang_end(2 * chrono.CH_C_PI)  # Full circle
path.Set_center(chrono.ChVectorD(0, 0, terrainHeight))
path.Set_plane_dir(chrono.ChVectorD(0, 0, 1))  # XY plane
path.Set_plane_uv(chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))
path.Set_radius(path_radius)

# Create path follower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  # Constant throttle
driver.SetSteeringPID(2.0, 0.0, 0.1)  # PID gains for steering

# Add visualization spheres for target and sentinel points
target_sphere = chrono.ChSphereShape()
target_sphere.GetSphereGeometry().rad = 0.3
target_sphere.SetColor(chrono.ChColor(1, 0, 0))  # Red for target
target_sphere.SetPos(chrono.ChVectorD(0, 0, terrainHeight))
vehicle.GetSystem().AddVisualizationShape(target_sphere)

sentinel_sphere = chrono.ChSphereShape()
sentinel_sphere.GetSphereGeometry().rad = 0.3
sentinel_sphere.SetColor(chrono.ChColor(0, 1, 0))  # Green for sentinel
sentinel_sphere.SetPos(chrono.ChVectorD(0, 0, terrainHeight))
vehicle.GetSystem().AddVisualizationShape(sentinel_sphere)

driver.Initialize()

# Simulation loop variables
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

    # Update driver and retrieve inputs
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    # Update visualization spheres
    target_point = driver.GetTargetPoint()
    sentinel_point = driver.GetSentinelPoint()
    target_sphere.SetPos(target_point)
    sentinel_sphere.SetPos(sentinel_point)

    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)