import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# *** 1. Initial vehicle position – changed from (0,0,0.5) to (-40,0,0.5) ***
initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # not used directly
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

# *** 2. Double‑lane‑change maneuver parameters ***
# Define the time intervals (seconds) for the maneuver
t_lane1_start = 1.0   # start of first lane change
t_lane1_end   = 2.0   # end of first lane change
t_lane2_start = 3.0   # start of second lane change
t_lane2_end   = 4.0   # end of second lane change
t_brake_start = 5.0   # start of braking

def get_driver_inputs(t):
    """Return (throttle, steering, braking) for a double lane‑change maneuver."""
    if t < t_lane1_start:
        # Initial straight driving
        throttle = 0.6
        steering = 0.0
        braking  = 0.0
    elif t < t_lane1_end:
        # First lane change – steer left
        throttle = 0.6
        steering = -0.6
        braking  = 0.0
    elif t < t_lane2_start:
        # Return to centre – steer right
        throttle = 0.6
        steering = 0.6
        braking  = 0.0
    elif t < t_lane2_end:
        # Second lane change – steer left
        throttle = 0.6
        steering = -0.6
        braking  = 0.0
    elif t < t_brake_start:
        # Return to centre – steer right
        throttle = 0.6
        steering = 0.6
        braking  = 0.0
    else:
        # Braking phase
        throttle = 0.0
        steering = 0.0
        braking  = 0.8
    return throttle, steering, braking

# Create the UAZBUS vehicle, set parameters, and initialize
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# *** 3. Set collision system type BEFORE initialization (correct order) ***
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)

# *** 4. Terrain texture – changed from tile4.jpg to concrete.jpg ***
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
steering_time = 1.0   # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0   # time to go from 0 to +1
braking_time  = 0.3   # time to go from 0 to +1
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

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV‑Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs (from keyboard/wheel)
    driver_inputs = driver.GetInputs()

    # *** 5. Override driver inputs with the double‑lane‑change maneuver ***
    throttle, steering, braking = get_driver_inputs(time)
    driver_inputs.throttle = throttle
    driver_inputs.steering = steering
    driver_inputs.braking  = braking

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