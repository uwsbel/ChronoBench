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
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # Increased terrain length to 200.0
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

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
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
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create circular path (approximated with Bezier curve)
radius = 10.0
points = []
num_points = 8  # Number of points for circle approximation
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    points.append(chrono.ChVector3d(x, y, 0.5))
path = chrono.ChBezierCurve(points)

# Create path visualization balls
ball_radius = 1.0
path_ball1 = chrono.ChBody()
path_ball1.SetPos(chrono.ChVector3d(radius, 0, 0.5))
path_ball1.SetFixed(True)
path_ball1.SetCollide(False)
sphere_shape1 = chrono.ChSphereShape(ball_radius)
sphere_shape1.SetColor(chrono.ChColor(1, 0, 0))
path_ball1.AddVisualShape(sphere_shape1)
vehicle.GetSystem().AddBody(path_ball1)

path_ball2 = chrono.ChBody()
path_ball2.SetPos(chrono.ChVector3d(-radius, 0, 0.5))
path_ball2.SetFixed(True)
path_ball2.SetCollide(False)
sphere_shape2 = chrono.ChSphereShape(ball_radius)
sphere_shape2.SetColor(chrono.ChColor(0, 1, 0))
path_ball2.AddVisualShape(sphere_shape2)
vehicle.GetSystem().AddBody(path_ball2)

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create path follower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "my_path", 5.0)
steering_controller = driver.GetSteeringController()
steering_controller.SetGains(Kp=0.5, Ki=0.0, Kd=0.1)
driver.GetSpeedController().SetGains(Kp=0.4, Ki=0.0, Kd=0.0)
driver.Initialize()

# Create controller visualization spheres
sentinel_ball = chrono.ChBody()
sentinel_ball.SetFixed(True)
sentinel_ball.SetCollide(False)
sentinel_shape = chrono.ChSphereShape(0.5)
sentinel_shape.SetColor(chrono.ChColor(0, 0, 1))
sentinel_ball.AddVisualShape(sentinel_shape)
vehicle.GetSystem().AddBody(sentinel_ball)

target_ball = chrono.ChBody()
target_ball.SetFixed(True)
target_ball.SetCollide(False)
target_shape = chrono.ChSphereShape(0.5)
target_shape.SetColor(chrono.ChColor(1, 1, 0))
target_ball.AddVisualShape(target_shape)
vehicle.GetSystem().AddBody(target_ball)

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

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs and set constant throttle
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  # Constant throttle value

    # Update controller visualization positions
    sentinel_pos = steering_controller.GetSentinelPosition()
    target_pos = steering_controller.GetTargetPosition()
    sentinel_ball.SetPos(sentinel_pos)
    target_ball.SetPos(target_pos)

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