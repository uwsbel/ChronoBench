import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set the path to the Chrono data directory
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
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() 
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(1e-3)
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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create a circular path
path_radius = 50.0
path_center = chrono.ChVector3d(0, 0, 0)

# Visualize the path using two balls
ball1 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z + 1.0), chrono.ChColor(1, 0, 0))
ball2 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, chrono.ChVector3d(path_center.x - path_radius, path_center.y, path_center.z + 1.0), chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(ball1)
vehicle.GetSystem().Add(ball2)

# PID controller gains
Kp = 10.0
Ki = 0.1
Kd = 0.1

# Target point
target_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)

# Sentinel point
sentinel_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)

# Visualize the target and sentinel points
target_sphere = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, target_point, chrono.ChColor(0, 0, 1))
sentinel_sphere = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, sentinel_point, chrono.ChColor(1, 1, 0))
vehicle.GetSystem().Add(target_sphere)
vehicle.GetSystem().Add(sentinel_sphere)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Constant throttle value
throttle = 0.3

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % math.ceil(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update target point
    target_point = chrono.ChVector3d(path_center.x + path_radius * math.cos(time), path_center.y + path_radius * math.sin(time), path_center.z)
    target_sphere.SetPos(target_point)

    # Update sentinel point
    sentinel_point = chrono.ChVector3d(path_center.x + path_radius * math.cos(time + math.pi / 2), path_center.y + path_radius * math.sin(time + math.pi / 2), path_center.z)
    sentinel_sphere.SetPos(sentinel_point)

    # Calculate steering angle using PID controller
    error = (target_point - vehicle.GetChassisBody().GetPos()).Length()
    steering_angle = Kp * error + Ki * error * step_size + Kd * (error - (target_point - vehicle.GetChassisBody().GetPos()).Length()) / step_size

    # Update vehicle
    vehicle.GetSteering().SetSteering(steering_angle)
    vehicle.GetTransmission().SetThrottle(throttle)

    # Advance simulation for one timestep for all modules
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)