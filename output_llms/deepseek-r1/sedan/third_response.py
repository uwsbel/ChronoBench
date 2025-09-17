import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVector3d(0, 0, 1.0)  # Increased Z for better clearance
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes (decreased for finer control)
step_size = 5e-4  # Reduced from 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 100  # Increased to 100 FPS

# Create vehicle
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
# Use highway mesh instead of box
patch = terrain.AddMesh(patch_mat, 
                        veh.GetDataFile("terrain/meshes/highway.obj"),
                        100,  # Scale factor
                        0,    # Height offset
                        chrono.ChVector3d(0, 0, 0), 
                        chrono.QUNIT)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan with PID Control')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set driver parameters (steering response increased to 5 seconds)
steering_time = 5.0  # Increased from 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# PID controller parameters
ref_speed = 10.0  # Reference speed (m/s)
Kp_throttle = 0.5  # Proportional gain for throttle
Ki_throttle = 0.1  # Integral gain for throttle
Kp_brake = 0.5     # Proportional gain for brake
integral = 0.0     # Integral error accumulator

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

    # Get base driver inputs (steering preserved)
    driver_inputs = driver.GetInputs()
    
    # PID speed control (override throttle/braking)
    speed = vehicle.GetVehicle().GetSpeed()
    error = ref_speed - speed
    
    if error > 0:  # Too slow - apply throttle
        integral += error * step_size
        throttle = Kp_throttle * error + Ki_throttle * integral
        driver_inputs.m_throttle = min(throttle, 1.0)
        driver_inputs.m_braking = 0.0
    else:  # Too fast - apply brakes
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_braking = min(Kp_brake * -error, 1.0)
        integral = 0  # Reset integral to prevent windup

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
    
    # Increment counters
    step_number += 1
    realtime_timer.Spin(step_size)