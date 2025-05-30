import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation - MODIFIED: changed from (0, 0, 0.5) to (-15, 0, 0.5)
initLoc = chrono.ChVector3d(-15, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # CORRECTED: ChQuaterniond -> ChQuaternionD

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

# Point chassis tracked by the camera - MODIFIED: changed from (0,0, 2.1) to (3,0, 2.1)
trackPoint = chrono.ChVector3d(3, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle, set parameters, and initialize
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)  # ADDED: Initialize tire model
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
# MODIFIED: changed chase camera distance from 25.0, 1.5 to 25.0, 10.5
vis.SetChaseCamera(trackPoint, 25.0, 10.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

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
print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# ADDED: Double lane change maneuver parameters
def double_lane_change_steering(time):
    """
    Implements a double lane change maneuver based on simulation time
    Returns steering input value between -1 and 1
    """
    # Define maneuver timing
    start_time = 2.0    # Start maneuver after 2 seconds
    phase1_end = start_time + 2.0   # First turn (left)
    phase2_end = phase1_end + 4.0   # Return to center and turn right
    phase3_end = phase2_end + 2.0   # Return to center
    
    if time < start_time:
        return 0.0  # Go straight initially
    elif time < phase1_end:
        # First lane change (turn left)
        progress = (time - start_time) / (phase1_end - start_time)
        return -0.3 * math.sin(progress * math.pi)
    elif time < phase2_end:
        # Second lane change (turn right)
        progress = (time - phase1_end) / (phase2_end - phase1_end)
        return 0.6 * math.sin(progress * math.pi)
    elif time < phase3_end:
        # Return to straight
        progress = (time - phase2_end) / (phase3_end - phase2_end)
        return -0.3 * math.sin(progress * math.pi)
    else:
        return 0.0  # Go straight after maneuver

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
    
    # ADDED: Override steering input with double lane change maneuver
    maneuver_steering = double_lane_change_steering(time)
    driver_inputs.m_steering = maneuver_steering
    driver_inputs.m_throttle = 0.3  # Constant throttle for forward motion
    driver_inputs.m_braking = 0.0   # No braking during maneuver

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

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)