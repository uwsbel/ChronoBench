import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ===================================================
# Implement requested modifications
# ===================================================

# 1. Adjusted initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)  # Increased height to prevent sinking
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# 2. Increased steering response time to 5 seconds
steering_time = 5.0  # Changed from 1.0 to 5.0

# 3. Decreased step sizes for finer control
step_size = 2e-4  # Reduced from 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 100  # Increased rendering frequency (100 FPS)

# 4. Terrain initialized with highway mesh
terrain_mesh_file = "terrain/meshes/highway.obj"  # Highway mesh definition

# 5. Reference speed for controller (10 m/s = 36 km/h)
reference_speed = 10.0

# 6. PID controller parameters
Kp = 0.5  # Proportional gain
Ki = 0.1  # Integral gain
Kd = 0.0  # Derivative gain
prev_error = 0.0
integral = 0.0

# ===================================================
# Corrected and modified script
# ===================================================

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
contact_method = chrono.ChContactMethod_NSC
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Create vehicle
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

# Create highway mesh terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,  # Default orientation
    veh.GetDataFile(terrain_mesh_file)  # Highway mesh file
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 with Speed Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / 1.0)  # Keep default throttle response
driver.SetBrakingDelta(render_step_size / 0.3)   # Keep default braking response
driver.Initialize()

# Simulation setup
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Main simulation loop
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
    
    # ===================================================
    # Implement PID speed controller
    # ===================================================
    current_speed = vehicle.GetVehicle().GetSpeed()
    error = reference_speed - current_speed
    
    # PID calculations
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    throttle_control = Kp * error + Ki * integral + Kd * derivative
    
    # Clamp and apply control signal
    throttle_control = chrono.Clamp(throttle_control, 0.0, 1.0)
    driver_inputs.m_throttle = throttle_control
    prev_error = error

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(f"{time:.2f} s | Speed: {current_speed:.1f} m/s | Throttle: {throttle_control:.2f}", 
                    driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)