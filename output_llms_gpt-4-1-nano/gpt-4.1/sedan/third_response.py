import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# 1. Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Initial vehicle location and orientation adjusted (as per instruction)
initLoc = chrono.ChVector3d(5, 2, 0.6)  # Example: moved 5m forward, 2m right, 0.6m up
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# 3. Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# 4. Terrain parameters (will use mesh, not box)
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# 5. Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# 6. Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# 7. Simulation step sizes (decreased for finer control)
step_size = 5e-4          # was 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 100  # FPS = 100, was 1.0/50

# 8. Create the Sedan vehicle, set parameters, and initialize
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

# 9. Terrain: Initialize with a highway mesh (as per instruction)
terrain = veh.RigidTerrain(vehicle.GetSystem())
# Use a sample mesh from Chrono data (e.g., "highway.obj")
mesh_file = veh.GetDataFile("terrain/Highway/highway.obj")
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, mesh_file, chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 1, 1)
terrain.Initialize()

# 10. Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# 11. Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# 12. Set the time response for steering and throttle keyboard inputs.
steering_time = 5.0  # Increased to 5 seconds (was 1.0)
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# 13. Reference speed input and PID controller for throttle
reference_speed = 10.0  # m/s, desired speed
Kp = 0.5                # Proportional gain
Ki = 0.1                # Integral gain
Kd = 0.0                # Derivative gain

integral_error = 0.0
prev_error = 0.0

# 14. Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# 15. Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# 16. Simulation loop variables
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

    # Get driver inputs (steering, throttle, braking)
    driver_inputs = driver.GetInputs()

    # --- PID controller for throttle ---
    # Get current speed (longitudinal, in vehicle frame)
    veh_speed = vehicle.GetVehicle().GetSpeed()
    speed_error = reference_speed - veh_speed
    integral_error += speed_error * step_size
    derivative_error = (speed_error - prev_error) / step_size
    prev_error = speed_error

    # PID output for throttle
    throttle_output = Kp * speed_error + Ki * integral_error + Kd * derivative_error
    throttle_output = max(0.0, min(throttle_output, 1.0))  # Clamp between 0 and 1

    # If reference speed is negative, apply braking instead (not used here)
    braking_output = 0.0
    if reference_speed < 0.1 and veh_speed < 0.1:
        throttle_output = 0.0
        braking_output = 1.0

    # Override driver throttle/brake with PID output
    driver_inputs.m_throttle = throttle_output
    driver_inputs.m_braking = braking_output

    # --- Synchronize modules ---
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # --- Advance simulation ---
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)