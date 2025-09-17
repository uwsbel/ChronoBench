import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set this path before running the demo!
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation adjusted
initLoc = chrono.ChVector3d(0, 0, 0.6)
initRot = chrono.ChQuaterniond()
initRot.Q_from_AngAxis(math.pi / 6, chrono.ChVector3d(0, 1, 0))  # 30-degree yaw

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 200.0  # Increased for highway
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes decreased for finer control
step_size = 5e-4
tire_step_size = step_size
render_step_size = 1.0 / 100  # Increased FPS to 100

# PID controller parameters
target_speed = 30.0  # Reference speed in m/s
Kp = 0.5
Ki = 0.05
Kd = 0.01
integral = 0.0
prev_error = 0.0

# ------------------
# Create systems
# ------------------

# Create the Sedan vehicle, set parameters, and initialize
vehicle = veh.BMW_E90()
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

# Create the terrain with a highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Load highway mesh for terrain visualization
patch.SetMesh(veh.GetDataFile("terrain/meshes/highway.obj"), 
              chrono.ChVector3d(1, 1, 1), 
              chrono.ChVector3d(0, 0, 0), 
              True, True)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))  # Neutral color for mesh
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

# Increased steering response time to 5 seconds
steering_time = 5.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# -------------------
# Simulation loop
# -------------------

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between renders
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
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

    # PID throttle control implementation
    current_speed = vehicle.GetVehicle().GetSpeed()
    speed_error = target_speed - current_speed
    integral += speed_error * step_size
    integral = max(-1.0, min(1.0, integral))  # Anti-windup
    derivative = (speed_error - prev_error) / step_size if step_size > 0 else 0.0
    throttle = Kp * speed_error + Ki * integral + Kd * derivative
    throttle = max(0.0, min(1.0, throttle))  # Clamp output
    prev_error = speed_error
    driver_inputs.m_throttle = throttle  # Override throttle input

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

    # Maintain real-time execution
    realtime_timer.Spin(step_size)