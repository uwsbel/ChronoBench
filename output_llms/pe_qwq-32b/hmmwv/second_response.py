import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correct data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + '/vehicle/')  # Added slash for correct path

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 200.0  # Increased to 200.0
terrainWidth = 100.0

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Visualization settings
render_step_size = 1.0 / 50  # FPS = 50

# Circular path parameters
path_radius = 10.0  # Circular path radius
num_path_points = 36  # Number of points for the circular path

# PID controller gains for steering
kp_steering = 0.5
ki_steering = 0.0
kd_steering = 0.1

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
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

# Create circular path
path = chrono.ChPath()
for i in range(num_path_points):
    angle = 2 * math.pi * i / num_path_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path.AddPoint(chrono.ChVectorD(x, y, terrainHeight))

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add sentinel and target spheres for path visualization
sentinel_sphere = chrono.ChBodyEasySphere(0.2, 1000)
sentinel_sphere.SetBodyFixed(True)
sentinel_sphere.SetCollide(False)
sentinel_sphere.SetPos(path.GetPoint(0))  # Initialize at first path point
vehicle.GetSystem().AddBody(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.2, 1000)
target_sphere.SetBodyFixed(True)
target_sphere.SetCollide(False)
target_sphere.SetPos(path.GetPoint(1))  # Initialize at second path point
vehicle.GetSystem().AddBody(target_sphere)

# Create path follower driver with PID controller
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  # Constant throttle
driver.SetSteeringPIDGains(kp_steering, ki_steering, kd_steering)  # Set PID gains
driver.Initialize()

# Simulation loop setup
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Update sentinel and target positions dynamically
    current_pos = vehicle.GetVehicle().GetChassis().GetPos()
    closest_point = path.FindClosestPoint(current_pos)
    next_point_index = path.FindSegment(current_pos) + 1  # Get next segment index
    next_point = path.GetPoint(next_point_index % num_path_points)
    sentinel_sphere.SetPos(closest_point)
    target_sphere.SetPos(next_point)

    # Render scene
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update driver inputs (now path-following, no interactive inputs)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

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

# Correct data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + '/vehicle/')  # Added slash for correct path

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 200.0  # Increased to 200.0
terrainWidth = 100.0

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Visualization settings
render_step_size = 1.0 / 50  # FPS = 50

# Circular path parameters
path_radius = 10.0  # Circular path radius
num_path_points = 36  # Number of points for the circular path

# PID controller gains for steering
kp_steering = 0.5
ki_steering = 0.0
kd_steering = 0.1

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
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

# Create circular path
path = chrono.ChPath()
for i in range(num_path_points):
    angle = 2 * math.pi * i / num_path_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path.AddPoint(chrono.ChVectorD(x, y, terrainHeight))

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add sentinel and target spheres for path visualization
sentinel_sphere = chrono.ChBodyEasySphere(0.2, 1000)
sentinel_sphere.SetBodyFixed(True)
sentinel_sphere.SetCollide(False)
sentinel_sphere.SetPos(path.GetPoint(0))  # Initialize at first path point
vehicle.GetSystem().AddBody(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.2, 1000)
target_sphere.SetBodyFixed(True)
target_sphere.SetCollide(False)
target_sphere.SetPos(path.GetPoint(1))  # Initialize at second path point
vehicle.GetSystem().AddBody(target_sphere)

# Create path follower driver with PID controller
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  # Constant throttle
driver.SetSteeringPIDGains(kp_steering, ki_steering, kd_steering)  # Set PID gains
driver.Initialize()

# Simulation loop setup
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Update sentinel and target positions dynamically
    current_pos = vehicle.GetVehicle().GetChassis().GetPos()
    closest_point = path.FindClosestPoint(current_pos)
    next_point_index = path.FindSegment(current_pos) + 1  # Get next segment index
    next_point = path.GetPoint(next_point_index % num_path_points)
    sentinel_sphere.SetPos(closest_point)
    target_sphere.SetPos(next_point)

    # Render scene
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update driver inputs (now path-following, no interactive inputs)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)