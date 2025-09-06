import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted)
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes (decreased for finer control)
step_size = 5e-4  # Reduced from 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 100  # Increased to 100 FPS

# --------------
# Create systems
# --------------

# Create the Sedan vehicle
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

# Create the terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Load highway mesh
mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddMeshPatch(
    patch_mat,
    mesh_file,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(100, 100, 1),
    False,
    True
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 Highway Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set steering response time to 5 seconds (increased)
steering_time = 5.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Speed control parameters
target_speed = 10.0  # m/s (36 km/h)
kp = 0.5  # Proportional gain
ki = 0.1  # Integral gain
kd = 0.01  # Derivative gain
integral = 0.0
prev_error = 0.0

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
    
    # PID speed controller
    current_speed = vehicle.GetVehicle().GetSpeed()
    error = target_speed - current_speed
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    throttle_control = kp * error + ki * integral + kd * derivative
    
    # Clamp and apply throttle control
    throttle_control = chrono.Clamp(throttle_control, 0.0, 1.0)
    driver_inputs.m_throttle = throttle_control
    prev_error = error

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
    realtime_timer.Spin(step_size)