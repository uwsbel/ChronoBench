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
terrainLength = 200.0  # size in X direction
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
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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

# Path following setup
radius = 20.0
path_points = [chrono.ChVector3d(radius * math.cos(t), radius * math.sin(t), 0.5) for t in
               [0, math.pi / 4, math.pi / 2, 3 * math.pi / 4, math.pi, 5 * math.pi / 4, 3 * math.pi / 2, 7 * math.pi / 4]]

# PID controller gains
Kp = 1.0
Ki = 0.1
Kd = 0.01

# Initialize PID controller
pid = chrono.ChPIDController(Kp, Ki, Kd)
pid.SetTarget(0.0)

# Visualization of path points
sentinel_sphere = chrono.ChSphereShape()
sentinel_sphere.GetMaterialSurfaceNSC().SetFriction(0.9)
sentinel_sphere.GetMaterialSurfaceNSC().SetRestitution(0.01)
sentinel_sphere.SetRadius(0.2)
sentinel_sphere.SetColor(chrono.ChColor(1, 0, 0))
sentinel = chrono.ChBody()
sentinel.AddAsset(sentinel_sphere)
vehicle.GetSystem().Add(sentinel)

target_sphere = chrono.ChSphereShape()
target_sphere.GetMaterialSurfaceNSC().SetFriction(0.9)
target_sphere.GetMaterialSurfaceNSC().SetRestitution(0.01)
target_sphere.SetRadius(0.2)
target_sphere.SetColor(chrono.ChColor(0, 1, 0))
target = chrono.ChBody()
target.AddAsset(target_sphere)
vehicle.GetSystem().Add(target)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
current_target_index = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update target and sentinel positions
    target.SetPos(path_points[current_target_index])
    sentinel.SetPos(vehicle.GetVehicle().GetChassis().GetPos())

    # Calculate steering angle using PID controller
    error = vehicle.GetVehicle().GetChassis().GetPos().x() - target.GetPos().x()
    pid.SetInput(error)
    steering_angle = pid.GetOutput()

    # Apply constant throttle and steering
    vehicle.GetVehicle().SetThrottle(0.3)
    vehicle.GetVehicle().SetSteering(steering_angle)

    # Advance simulation
    terrain.Synchronize(time)
    vehicle.Synchronize(time)
    vis.Synchronize(time)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Switch to next target point when reached
    if (vehicle.GetVehicle().GetChassis().GetPos() - target.GetPos()).Length() < 1:
        current_target_index = (current_target_index + 1) % len(path_points)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)