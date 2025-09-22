import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation - modified as requested
initLoc = chrono.ChVector3d(-15, 0, 0.5)
# Adjusted the initial rotation to still face forward (no change needed as it was already unit quaternion)
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

# Point on chassis tracked by the camera - modified as requested
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
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
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

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
# Modified chase camera parameters as requested
vis.SetChaseCamera(trackPoint, 25.0, 10.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())  # Corrected: Use GetVehicle() instead of GetTractor()

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
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())  # Corrected: Use GetVehicle() instead of GetTractor()

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter and timer
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Parameters for the double lane change maneuver
lane_change_start_time = 3.0  # When to start the first lane change
first_lane_change_duration = 2.0  # Duration of first lane change (move right)
straight_duration = 3.0  # Duration of straight driving between lane changes
second_lane_change_duration = 2.0  # Duration of second lane change (move left)
steering_amplitude = 0.5  # Maximum steering angle during lane change

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
    
    # Implement double lane change maneuver
    # Override driver inputs based on the simulation time
    if time >= lane_change_start_time:
        # Set a constant throttle throughout the maneuver
        driver_inputs.m_throttle = 0.4
        
        # First lane change (move right)
        if time < lane_change_start_time + first_lane_change_duration:
            phase = (time - lane_change_start_time) / first_lane_change_duration
            # Sine wave for smooth steering
            if phase < 0.5:
                driver_inputs.m_steering = steering_amplitude * math.sin(phase * math.pi)
            else:
                driver_inputs.m_steering = -steering_amplitude * math.sin((phase - 0.5) * math.pi)
        
        # Straight driving between lane changes
        elif time < lane_change_start_time + first_lane_change_duration + straight_duration:
            driver_inputs.m_steering = 0.0
        
        # Second lane change (move left)
        elif time < lane_change_start_time + first_lane_change_duration + straight_duration + second_lane_change_duration:
            phase = (time - (lane_change_start_time + first_lane_change_duration + straight_duration)) / second_lane_change_duration
            # Sine wave for smooth steering in opposite direction
            if phase < 0.5:
                driver_inputs.m_steering = -steering_amplitude * math.sin(phase * math.pi)
            else:
                driver_inputs.m_steering = steering_amplitude * math.sin((phase - 0.5) * math.pi)
        
        # Return to straight driving after completing the maneuver
        else:
            driver_inputs.m_steering = 0.0

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