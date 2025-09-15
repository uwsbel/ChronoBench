import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0.0  # terrain height
terrainLength = 100.0   # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 0.0001  # finer control
tire_step_size = step_size
render_step_size = step_size  # Set to match simulation step for consistent rendering

# Create systems
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Create grid of patches for highway-like terrain
patches = []
for i in range(int(terrainLength / 2) + 1):
    for j in range(int(terrainWidth / 2) + 1):
        patch = terrain.AddPatch(patch_mat,
            chrono.ChCoordsysd(chrono.ChVector3d(i, j, 0), chrono.QUNIT),
            5, 5)  # Create smaller patches for a detailed highway
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        patches.append(patch)

terrain.Initialize()

# Create vehicle visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set PID controller parameters
kp = 0.1
ki = 0.01
kd = 0.1
ref_speed = 25.0  # 25 m/s reference speed (90 km/h)

# Set time responses
steering_time = 5.0  # Increased to 5 seconds
throttle_time = 0.5  # Decreased to allow faster throttle response
braking_time = 0.3

# Set input deltas
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)
render_frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Calculate speed error
    speed = vehicle.GetVehicle().GetSpeed()
    speed_error = ref_speed - speed

    # PID controller for throttle
    if abs(speed_error) > 0.01:
        throttle = (kp * speed_error) + (ki * sum(speed_errors) * step_size) + (kd * (speed_error - speed_errors[-1]))
        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0.0:
            throttle = 0.0
        vehicle.SetThrottle(throttle)
    
    # Store speed error for PID
    speed_errors.append(speed_error)

    step_number += 1
    realtime_timer.Spin(step_size)