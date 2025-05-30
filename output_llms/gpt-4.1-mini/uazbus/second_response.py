import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path only if it's not empty, else error or set manually
chrono_data_path = chrono.GetChronoDataPath()
if not chrono_data_path:
    raise Exception("Chrono data path is not set. Please set CHRONO_DATA_PATH environment variable or set path manually.")
chrono.SetChronoDataPath(chrono_data_path)
veh.SetDataPath(chrono_data_path + 'vehicle/')

# Initial vehicle location and orientation
# Changed initial position as per instructions
initLoc = chrono.ChVector3d(-40, 0, 0.5)
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

# Initialize vehicle before setting visualization types
vehicle.Initialize()

# Set visualization (can be set before or after Initialize, but better to set before or immediately after)
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type to BULLET (optional but good practice)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)

# Changed texture to concrete.jpg as per instructions
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

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter and realtime timer
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Additional variables for double lane change maneuver
# Define time points (in seconds) for maneuver phases
t_start = 0.5     # start time to do any steering
t_first_lane_change_start = 1.0
t_first_lane_change_end = 3.0
t_return_lane_start = 3.0
t_return_lane_end = 5.0
t_second_lane_change_start = 5.0
t_second_lane_change_end = 7.0
t_braking_start = 7.0
t_braking_end = 10.0

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Implement double lane change maneuver logic:
    steering_input = 0.0
    throttle_input = 0.8  # nominal throttle (80%)
    braking_input = 0.0

    if t_first_lane_change_start <= time < t_first_lane_change_end:
        # Steer right for first lane change
        # Steering smoothly increases to about 0.5 right
        steering_input = 0.5 * (time - t_first_lane_change_start) / (t_first_lane_change_end - t_first_lane_change_start)
    elif t_return_lane_start <= time < t_return_lane_end:
        # Steer left to return to original lane
        steering_input = 0.5 * (1 - (time - t_return_lane_start) / (t_return_lane_end - t_return_lane_start))
    elif t_second_lane_change_start <= time < t_second_lane_change_end:
        # Steer right again for second lane change
        steering_input = 0.5 * (time - t_second_lane_change_start) / (t_second_lane_change_end - t_second_lane_change_start)
    elif t_braking_start <= time < t_braking_end:
        # Apply braking gradually and reduce throttle to zero
        steering_input = 0.0
        throttle_input = max(0, 0.8 * (1 - (time - t_braking_start) / (t_braking_end - t_braking_start)))
        braking_input = (time - t_braking_start) / (t_braking_end - t_braking_start)
    elif time >= t_braking_end:
        # Full brake after maneuver finished
        steering_input = 0.0
        throttle_input = 0.0
        braking_input = 1.0

    # Clamp inputs to [-1, 1]
    steering_input = max(-1.0, min(1.0, steering_input))
    throttle_input = max(0.0, min(1.0, throttle_input))
    braking_input = max(0.0, min(1.0, braking_input))

    # Pass driver inputs manually to vehicle (override driver.GetInputs())
    # The driver object here is interactive from Irrlicht, but we override inputs
    # So we create an inputs structure to synchronize with vehicle

    # Using driver inputs structure to feed custom inputs:
    # Create a custom inputs structure based on PyChrono standard interface:
    class CustomDriverInputs:
        def __init__(self, steering, throttle, braking):
            self.m_steering = steering
            self.m_throttle = throttle
            self.m_braking = braking

        def GetSteering(self):
            return self.m_steering

        def GetThrottle(self):
            return self.m_throttle

        def GetBraking(self):
            return self.m_braking

    custom_inputs = CustomDriverInputs(steering_input, throttle_input, braking_input)

    # Synchronize modules with time and custom inputs
    driver.Synchronize(time)
    terrain.Synchronize(time)
    # Use custom inputs instead of driver.GetInputs() to synchronize vehicle
    vehicle.Synchronize(time, custom_inputs, terrain)
    vis.Synchronize(time, custom_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)