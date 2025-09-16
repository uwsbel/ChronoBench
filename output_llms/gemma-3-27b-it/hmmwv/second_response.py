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
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0
terrainLength = 200.0  # Increased terrain length
terrainWidth = 100.0

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
# veh.HMMWV_Reduced() could be another choice here
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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength,
                         terrainWidth)
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

# Path following parameters
radius = 20.0
center = chrono.ChVector3d(50, 0, 0)
num_points = 36  # Number of points defining the circular path
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = center.x + radius * math.cos(angle)
    y = center.y + radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0))

# PID controller gains
kp = 1.0
ki = 0.0
kd = 0.1

# Initialize PID controller
integral = 0.0
previous_error = 0.0

# Visualization of path points
path_spheres = []
for point in path_points:
    sphere = chrono.ChBodyEasy()
    sphere.SetPos(point)
    sphere.SetMass(1.0)
    sphere.SetCollide(True)
    sphere.SetBodyFixed(False)
    sphere.AddVisualization("sphere", 0.5)
    vehicle.GetSystem().Add(sphere)
    path_spheres.append(sphere)

# Set constant throttle
throttle_value = 0.3

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
current_path_point_index = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Calculate error
    vehicle_pos = vehicle.GetVehicle().GetPos()
    target_point = path_points[current_path_point_index]
    error = target_point.x - vehicle_pos.x

    # PID control
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    steering_angle = kp * error + ki * integral + kd * derivative
    previous_error = error

    # Limit steering angle
    steering_angle = max(-0.5, min(steering_angle, 0.5))

    # Apply steering and throttle
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.throttle = throttle_value
    driver_inputs.steering = steering_angle
    driver_inputs.braking = 0.0

    # Update modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update path point index
    vehicle_pos = vehicle.GetVehicle().GetPos()
    target_point = path_points[current_path_point_index]
    distance_to_target = chrono.ChVector3d(target_point.x - vehicle_pos.x, target_point.y - vehicle_pos.y, 0).Norm()
    if distance_to_target < 2.0:
        current_path_point_index = (current_path_point_index + 1) % num_points

    step_number += 1
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1