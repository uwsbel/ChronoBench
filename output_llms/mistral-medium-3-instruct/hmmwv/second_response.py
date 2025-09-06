import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np

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
terrainLength = 200.0  # size in X direction (increased from 100 to 200)
terrainWidth = 200.0   # size in Y direction (increased from 100 to 200)

# Point on chassis tracked by the camera
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
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# =============================================
# Path and Controller Implementation
# =============================================

# Circular path parameters
path_radius = 25.0  # meters
path_center = chrono.ChVector3d(0, 0, 0)
throttle = 0.3

# PID controller parameters
Kp = 0.5  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.1  # Derivative gain

# Create visualization for the path
path_vis_radius = 0.5
path_vis_mat = chrono.ChContactMaterialNSC()
path_vis_mat.SetFriction(0.9)
path_vis_mat.SetRestitution(0.01)

# Create two balls to visualize the path
ball1 = chrono.ChBody()
ball1.SetPos(chrono.ChVector3d(path_radius, 0, 0.5))
ball1.SetRot(chrono.QUNIT)
ball1.SetMass(1)
ball1.SetCollide(False)
ball1.SetBodyFixed(True)
ball1.GetVisualModel().AddSphere(path_vis_radius)
ball1.GetVisualModel().SetColor(chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBody()
ball2.SetPos(chrono.ChVector3d(-path_radius, 0, 0.5))
ball2.SetRot(chrono.QUNIT)
ball2.SetMass(1)
ball2.SetCollide(False)
ball2.SetBodyFixed(True)
ball2.GetVisualModel().AddSphere(path_vis_radius)
ball2.GetVisualModel().SetColor(chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(ball2)

# Create visualization for sentinel and target points
sentinel_ball = chrono.ChBody()
sentinel_ball.SetPos(chrono.ChVector3d(0, 0, 0.5))
sentinel_ball.SetRot(chrono.QUNIT)
sentinel_ball.SetMass(1)
sentinel_ball.SetCollide(False)
sentinel_ball.SetBodyFixed(True)
sentinel_ball.GetVisualModel().AddSphere(0.3)
sentinel_ball.GetVisualModel().SetColor(chrono.ChColor(0, 0, 1))  # Blue for sentinel
vehicle.GetSystem().Add(sentinel_ball)

target_ball = chrono.ChBody()
target_ball.SetPos(chrono.ChVector3d(0, 0, 0.5))
target_ball.SetRot(chrono.QUNIT)
target_ball.SetMass(1)
target_ball.SetCollide(False)
target_ball.SetBodyFixed(True)
target_ball.GetVisualModel().AddSphere(0.3)
target_ball.GetVisualModel().SetColor(chrono.ChColor(1, 1, 0))  # Yellow for target
vehicle.GetSystem().Add(target_ball)

# PID controller variables
integral = 0
prev_error = 0

# =============================================
# Simulation Loop
# =============================================

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

    # Get current vehicle position and orientation
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_rot = vehicle.GetVehicle().GetRot()

    # Calculate current heading direction (vehicle's forward direction in world coordinates)
    heading = vehicle_rot.Rotate(chrono.ChVector3d(1, 0, 0))

    # Calculate vector from vehicle to path center
    to_center = path_center - vehicle_pos
    to_center.z = 0  # Ignore height

    # Calculate distance from vehicle to path center
    distance = to_center.Length()

    # Calculate desired heading direction (tangent to the circular path)
    if distance > 0.1:  # Avoid division by zero
        desired_heading = to_center.Cross(chrono.ChVector3d(0, 0, 1)).GetNormalized()
    else:
        desired_heading = chrono.ChVector3d(1, 0, 0)  # Default forward direction

    # Calculate the error angle between current heading and desired heading
    # We'll use the angle between the two vectors in the XY plane
    current_dir = chrono.ChVector3d(heading.x, heading.y, 0).GetNormalized()
    desired_dir = chrono.ChVector3d(desired_heading.x, desired_heading.y, 0).GetNormalized()

    # Calculate the angle between the two vectors
    dot = current_dir.Dot(desired_dir)
    cross = current_dir.Cross(desired_dir).z
    error = math.atan2(cross, dot)

    # PID controller
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    prev_error = error

    # Clamp steering to [-1, 1]
    steering = max(-1.0, min(1.0, steering))

    # Create driver inputs
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering
    driver_inputs.m_braking = 0

    # Update sentinel and target points visualization
    # Sentinel point is slightly ahead of the vehicle
    sentinel_pos = vehicle_pos + heading * 2.0
    sentinel_ball.SetPos(chrono.ChVector3d(sentinel_pos.x, sentinel_pos.y, 0.5))

    # Target point is along the desired path direction
    target_pos = vehicle_pos + desired_heading * 5.0
    target_ball.SetPos(chrono.ChVector3d(target_pos.x, target_pos.y, 0.5))

    # Update modules (process inputs from other modules)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())