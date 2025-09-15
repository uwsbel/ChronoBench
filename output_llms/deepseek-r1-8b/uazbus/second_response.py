import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

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


# Create the UAZBUS vehicle, set parameters, and initialize
vehicle = veh.UAZBUS() 
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
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
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
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Define time intervals for double lane change maneuver
lane_change_time = 3.0  # Total time for lane change maneuver
straight_time = 1.0    # Time spent going straight before lane change
lane_change_duration = 0.5  # Duration of lane change (90 degree turn)
deceleration_time = 0.5  # Time to decelerate after lane change
braking_time_total = 0.5  # Total braking time after lane change

# Calculate time points for each phase
change_point_time = straight_time + lane_change_duration
deceleration_start_time = change_point_time
braking_start_time = deceleration_start_time + deceleration_time

# Define steering and throttle adjustments
straight_steering = 0.0
max_steering = 0.5
lane_change_steering = max_steering  # 90 degree turn
deceleration_steering = 0.0
braking_steering = 0.0

straight_throttle = 0.5
lane_change_throttle = straight_throttle  # Maintain throttle during lane change
deceleration_throttle = 0.0
braking_throttle = 0.0

# Initialize time variables
current_time = 0.0
target_time = lane_change_time

def get_input(delta_time):
    global current_time, target_time
    current_time += delta_time
    
    # Straight movement
    if current_time < straight_time:
        steering = straight_steering
        throttle = straight_throttle
    # Lane change maneuver
    elif current_time >= straight_time and current_time < change_point_time:
        turn_angle = (current_time - straight_time) * (2 * lane_change_steering) / lane_change_duration
        steering = max_steering * (1 - math.sin(turn_angle / 2))
        throttle = lane_change_throttle
    # Deceleration phase
    elif current_time >= change_point_time and current_time < braking_start_time:
        steering = deceleration_steering
        throttle = deceleration_throttle
    # Braking phase
    else:
        steering = braking_steering
        throttle = braking_throttle
        
    return steering, throttle

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Get steering and throttle inputs for the vehicle
    steering_input, throttle_input = get_input(step_size)
    
    # Update driver inputs with new values
    driver.SetInputs(chrono.ChInputState(step_size, 
        steering_input, 
        0, 
        throttle_input,
        0,
        0))
    
    # Increment frame number
    step_number += 1
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)