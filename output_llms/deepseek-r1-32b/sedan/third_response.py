import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set this path before running the demo
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(5, 0, 0.5)  # Adjusted initial position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(5.0, 0.0, 1.8)  # Adjusted camera position

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-4  # Decreased simulation step size
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 60  # Increased render step size for finer control

# Reference speed for controlling vehicle's speed
target_speed = 20.0  # Target speed in m/s

# PID controller parameters
kp = 0.5
ki = 0.1
kd = 0.2
integral = 0.0
prev_error = 0.0

# --------------
# Create systems
# --------------

# Create the Sedan vehicle, set parameters, and initialize
vehicle = veh.Sedan()  # Changed to Sedan for better compatibility
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

# Initialize terrain with highway mesh
patch.SetTexture(veh.GetChronoDataFile('terrain/textures/highway.jpg'), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan with PID Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 5.0  # Increased steering response time
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)

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

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Calculate current speed and error for PID control
    current_speed = vehicle.GetVehicle().GetSpeed()
    error = target_speed - current_speed
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    pid_output = kp * error + ki * integral + kd * derivative
    pid_output = max(0.0, min(1.0, pid_output))  # Clamp between 0 and 1

    # Set throttle based on PID output
    driver_inputs.m_throttle = pid_output
    print(f"Current Speed: {current_speed:.2f} m/s | Throttle: {driver_inputs.m_throttle:.2f}")

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    prev_error = error