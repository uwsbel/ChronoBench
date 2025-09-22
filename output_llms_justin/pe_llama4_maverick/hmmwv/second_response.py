import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the Chrono data path
chrono.SetChronoDataPath('/path/to/chrono/data')  # Replace with the actual path
veh.SetDataPath('/path/to/vehicle/data')  # Replace with the actual path

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
terrainLength = 200.0  # size in X direction (increased from 100.0 to 200.0)
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

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Implement a circular path
path_radius = 30
path_center = chrono.ChVector3d(0, 0, 0)
num_path_points = 100
path_points = []
for i in range(num_path_points):
    angle = 2 * math.pi * i / num_path_points
    point = path_center + chrono.ChVector3d(path_radius * math.cos(angle), path_radius * math.sin(angle), 0)
    path_points.append(point)

# Visualize the path using two balls
path_ball1 = chrono.ChBodyEasySphere(1.0, 1000, True, True, patch_mat)
path_ball1.SetPos(path_points[0])
path_ball1.SetBodyFixed(True)
vehicle.GetSystem().Add(path_ball1)

path_ball2 = chrono.ChBodyEasySphere(1.0, 1000, True, True, patch_mat)
path_ball2.SetPos(path_points[num_path_points // 2])
path_ball2.SetBodyFixed(True)
vehicle.GetSystem().Add(path_ball2)

# Create a path-following driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path_points, 'my_path', 30, True)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0, 0)
driver.GetSpeedController().SetGains(0.4, 0, 0)
driver.SetDesiredSpeed(10.0)  # Adjust the desired speed as needed

# Set constant throttle value
throttle_value = 0.3

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
    driver_inputs = veh.Inputs()
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_steering = driver.GetSteering()
    driver_inputs.m_braking = 0

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Visualize the sentinel and target points
    sentinel_pos = driver.GetSteeringController().GetSentinelLocation()
    target_pos = driver.GetSteeringController().GetTargetLocation()

    sentinel_ball = chrono.ChBodyEasySphere(0.5, 1000, True, True, patch_mat)
    sentinel_ball.SetPos(sentinel_pos)
    vehicle.GetSystem().Add(sentinel_ball)

    target_ball = chrono.ChBodyEasySphere(0.5, 1000, True, True, patch_mat)
    target_ball.SetPos(target_pos)
    vehicle.GetSystem().Add(target_ball)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Remove the sentinel and target balls after rendering
    if (step_number % render_steps == 0):
        vehicle.GetSystem().Remove(sentinel_ball)
        vehicle.GetSystem().Remove(target_ball)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)