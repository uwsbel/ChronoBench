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

# Circular path parameters
path_radius = 20.0  # meters
path_center = chrono.ChVector3d(0, 0, 0)
constant_throttle = 0.3

# PID controller parameters
Kp = 0.5  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.1  # Derivative gain

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

# Create visualization for the circular path
path_vis = irr.ChIrrTools(vis.GetSceneManager(), vis.GetVideoDriver())
path_vis.addSphere(chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z), 0.5, chrono.ChColor(1, 0, 0))
path_vis.addSphere(chrono.ChVector3d(path_center.x - path_radius, path_center.y, path_center.z), 0.5, chrono.ChColor(0, 1, 0))

# PID controller variables
integral_error = 0
prev_error = 0

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get current vehicle position and orientation
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_rot = vehicle.GetVehicle().GetRot()

    # Calculate current heading (forward direction in XY plane)
    forward_dir = vehicle_rot.Rotate(chrono.ChVector3d(1, 0, 0))
    forward_dir.z = 0  # Project to XY plane
    forward_dir.Normalize()

    # Calculate vector from vehicle to path center
    to_center = path_center - vehicle_pos
    to_center.z = 0  # Project to XY plane

    # Calculate lateral error (distance from path center to vehicle's path)
    lateral_error = to_center.Cross(forward_dir).z

    # Calculate heading error (difference between desired and actual heading)
    desired_heading = to_center.Normalize()
    heading_error = math.atan2(desired_heading.Cross(forward_dir).z, desired_heading.Dot(forward_dir))

    # PID control for steering
    integral_error += lateral_error * step_size
    derivative_error = (lateral_error - prev_error) / step_size

    steering_angle = Kp * lateral_error + Ki * integral_error + Kd * derivative_error

    # Limit steering angle to reasonable values
    steering_angle = max(-0.5, min(0.5, steering_angle))

    prev_error = lateral_error

    # Create driver inputs
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = constant_throttle
    driver_inputs.m_steering = steering_angle
    driver_inputs.m_braking = 0

    # Visualize sentinel and target points
    if step_number % render_steps == 0:
        # Sentinel point (current target point on path)
        sentinel_pos = path_center + desired_heading * path_radius
        sentinel_vis = path_vis.addSphere(chrono.ChVector3d(sentinel_pos.x, sentinel_pos.y, sentinel_pos.z + 0.5),
                                         0.3, chrono.ChColor(0, 0, 1))

        # Target point (point ahead on path)
        target_pos = path_center + desired_heading * (path_radius + 5)
        target_vis = path_vis.addSphere(chrono.ChVector3d(target_pos.x, target_pos.y, target_pos.z + 0.5),
                                       0.3, chrono.ChColor(1, 1, 0))

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
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