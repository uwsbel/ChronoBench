import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.driver as driver
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation parameters
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vis_type = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
terrainHeight = 0
terrainLength = 200.0  # Increased terrain length
terrainWidth = 100.0
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50
radius = 10.0  # Circular path radius

# Create vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
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
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht interface
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
path = chrono.ChPath()
center = chrono.ChVectorD(0, 0, terrainHeight)
for theta in range(0, 360, 5):
    rad = math.radians(theta)
    x = center.x + radius * math.cos(rad)
    y = center.y + radius * math.sin(rad)
    tangent_x = -math.sin(rad)
    tangent_y = math.cos(rad)
    tangent = chrono.ChVectorD(tangent_x, tangent_y, 0)
    point = chrono.ChVectorD(x, y, terrainHeight)
    path.AddPoint(chrono.ChPathPoint(point, tangent))

# Create sentinel and target spheres
sentinel_sphere = chrono.ChBodyEasySphere(0.2, 1000)
sentinel_sphere.SetBodyFixed(True)
sentinel_sphere.SetPos(chrono.ChVectorD(0, 0, terrainHeight))
vehicle.GetSystem().AddBody(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.2, 1000)
target_sphere.SetBodyFixed(True)
target_sphere.SetPos(chrono.ChVectorD(radius, 0, terrainHeight))
vehicle.GetSystem().AddBody(target_sphere)

# Create path follower driver with PID controller
driver = driver.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  # Constant throttle
driver.SetSteeringPID(0.8, 0.1, 0.05)  # PID gains (Kp, Ki, Kd)
driver.Initialize()

# Simulation loop variables
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)  # Fixed parameter count
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sentinel and target positions
    current_param = driver.GetPath().GetCurrentParam()
    current_point = path.GetPoint(current_param)
    look_ahead_distance = 5.0  # meters
    target_param = current_param + look_ahead_distance
    path_length = path.GetLength()
    if target_param > path_length:
        target_param -= path_length  # Wrap around for circular path
    target_point = path.GetPoint(target_param)

    sentinel_sphere.SetPos(current_point)
    target_sphere.SetPos(target_point)

    step_number += 1
    realtime_timer.Spin(step_size)