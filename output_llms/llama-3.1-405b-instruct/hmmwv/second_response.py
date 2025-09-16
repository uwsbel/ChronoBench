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

# Path and Controller Implementation
# Implement a circular path with a reasonable radius.
# Visualize the path using two balls.
path_radius = 50.0
num_path_points = 100
path_points = []
for i in range(num_path_points):
    angle = 2 * math.pi * i / num_path_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0.5))

# Create the path visualization
path_vis = []
for point in path_points:
    path_vis.append(vis.AddSphere(point, 0.5, chrono.ChColor(1, 0, 0)))

# PID controller gains
kp = 0.1
ki = 0.01
kd = 0.1

# PID controller variables
error_prev = 0
integral = 0

# Constant throttle value
throttle = 0.3

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % int(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get the current vehicle position
    vehicle_pos = vehicle.GetVehicle().GetPos()

    # Find the closest point on the path to the vehicle
    closest_point = None
    closest_distance = float('inf')
    for point in path_points:
        distance = (vehicle_pos - point).Length()
        if distance < closest_distance:
            closest_distance = distance
            closest_point = point

    # Calculate the error
    error = (closest_point - vehicle_pos).Length()

    # Update the PID controller variables
    integral += error * step_size
    derivative = (error - error_prev) / step_size
    error_prev = error

    # Calculate the steering input
    steering = kp * error + ki * integral + kd * derivative

    # Update the vehicle inputs
    vehicle_inputs = veh.ChDriverInputs()
    vehicle_inputs.SetThrottle(throttle)
    vehicle_inputs.SetSteering(steering)

    # Update the vehicle
    vehicle.Synchronize(time, vehicle_inputs, terrain)

    # Advance the simulation
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    # Increment the frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)