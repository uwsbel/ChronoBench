import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-40, 0, 0.5)  # Changed from (0, 0, 0.5)
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

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Variables for lane change maneuver
# Define time intervals and steering/throttle targets
# For example:
# 0-2 sec: straight
# 2-4 sec: lane change left
# 4-6 sec: lane change right
# 6-8 sec: straighten
# 8-10 sec: brake
lane_change_schedule = [
    {"time": 2.0, "steering": 0.0, "throttle": 1.0},    # straight
    {"time": 4.0, "steering": -1.0, "throttle": 1.0},   # lane change left
    {"time": 6.0, "steering": 1.0, "throttle": 1.0},    # lane change right
    {"time": 8.0, "steering": 0.0, "throttle": 1.0},    # straighten
    {"time": 10.0, "steering": 0.0, "throttle": 0.0}    # brake
]

def get_driver_input(current_time):
    # Determine target steering and throttle based on schedule
    steering_target = 0.0
    throttle_target = 0.0
    for i in range(len(lane_change_schedule)):
        if current_time < lane_change_schedule[i]["time"]:
            if i == 0:
                # Before first scheduled change
                steering_target = lane_change_schedule[0]["steering"]
                throttle_target = lane_change_schedule[0]["throttle"]
            else:
                # Interpolate between previous and current schedule points
                t0 = lane_change_schedule[i-1]["time"]
                s0 = lane_change_schedule[i-1]["steering"]
                th0 = lane_change_schedule[i-1]["throttle"]
                t1 = lane_change_schedule[i]["time"]
                s1 = lane_change_schedule[i]["steering"]
                th1 = lane_change_schedule[i]["throttle"]
                ratio = (current_time - t0) / (t1 - t0)
                steering_target = s0 + ratio * (s1 - s0)
                throttle_target = th0 + ratio * (th1 - th0)
            break
    else:
        # After last schedule point
        steering_target = lane_change_schedule[-1]["steering"]
        throttle_target = lane_change_schedule[-1]["throttle"]
    return steering_target, throttle_target

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs based on schedule
    steering_target, throttle_target = get_driver_input(time)

    # Get current driver inputs
    driver_inputs = driver.GetInputs()

    # Compute delta inputs to smoothly follow target
    # For simplicity, assuming driver controls target steering and throttle directly
    # Here, we set driver inputs to target values directly for demonstration
    driver_inputs.m_steering = steering_target
    driver_inputs.m_throttle = throttle_target
    driver_inputs.m_braking = 0.0  # No braking unless specified in schedule

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)