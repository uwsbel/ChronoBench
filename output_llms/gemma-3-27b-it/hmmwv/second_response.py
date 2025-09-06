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
terrainHeight = 0  # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0  # size in Y direction

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
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
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

# Path following parameters
path_radius = 20.0
path_center = chrono.ChVector3d(100.0, 0.0, 0.0)
num_points = 36  # Number of points defining the circle
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = path_center.x + path_radius * math.cos(angle)
    y = path_center.y + path_radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0))

# PID controller gains
kp = 1.0
ki = 0.0
kd = 0.1

# Controller state
integral = 0.0
previous_error = 0.0

# Create visualization for path points
path_spheres = []
for point in path_points:
    sphere = chrono.ChBodyEasySphere(0.5, 1000, True, True, patch_mat)
    sphere.SetPos(point)
    sphere.SetFixed(True)
    vehicle.GetSystem().Add(sphere)
    path_spheres.append(sphere)

# Constant throttle value
throttle_value = 0.3

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
s = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Calculate the target point
    target_point = path_points[step_number % num_points]

    # Calculate the error
    error = math.atan2(target_point.y - vehicle.GetVehicle().GetPos().y, target_point.x - vehicle.GetVehicle().GetPos().x) - vehicle.GetVehicle().GetYaw()

    # PID control
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    steering_input = kp * error + ki * integral + kd * derivative
    previous_error = error

    # Limit steering input
    steering_input = max(-1.0, min(1.0, steering_input))

    # Apply throttle and steering
    vehicle.GetVehicle().SetThrottle(throttle_value)
    vehicle.GetVehicle().SetSteering(steering_input)

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time)
    vis.Synchronize(time)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    s.Spin(step_size)