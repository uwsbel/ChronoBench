import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# -------------------------------------
# Parameters you can change
# -------------------------------------

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-40, 0, 0.5)  # Modified initial position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction (extended for lane change)
terrainWidth = 200.0   # size in Y direction

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

# -------------------------------------
# Create the vehicle system
# -------------------------------------

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

# -------------------------------------
# Create the terrain
# -------------------------------------

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
                         terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"),  # Changed texture
                 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -------------------------------------
# Create the driver system
# -------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# -------------------------------------
# Simulation loop
# -------------------------------------

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Lane change parameters
lane_change_start_time = 5.0  # Start lane change after 5 seconds
lane_change_duration = 3.0    # Complete lane change in 3 seconds
braking_start_time = 15.0    # Start braking after 15 seconds

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Lane change logic
    if lane_change_start_time <= time <= lane_change_start_time + lane_change_duration:
        # Steer to the right lane
        steering_input = (time - lane_change_start_time) / lane_change_duration  
    elif time > braking_start_time:
        # Apply brakes
        throttle_input = 0.0
        braking_input = 1.0
    else:
        # Maintain lane
        steering_input = 0.0
        throttle_input = 0.3  # Maintain some speed
        braking_input = 0.0

    # Set driver inputs
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = steering_input
    driver_inputs.m_throttle = throttle_input
    driver_inputs.m_braking = braking_input

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

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

    # Increment frame number
    step_number += 1
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)