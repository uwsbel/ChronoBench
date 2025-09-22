import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

print(veh)
"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (adjusted position)
initLoc = chrono.ChVector3d(2, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes (fined for higher control)
step_size = 1e-4  # decreased from 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create systems
# Create the Sedan vehicle, set parameters, and initialize
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

# Initialize terrain with highway mesh
# Assuming 'crossfall.hmap' as a placeholder for highway mesh
# Note: For actual highway mesh, a height map or mesh file should be provided
hmap = chrono.ChHeightMapTerrain()
hmap.Initialize(terrainLength, terrainWidth, 0.0, 0.0, 0.0)  # Flat height map as placeholder
terrain.AddHighwayMesh(patch_mat, 
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
                       hmap)

# Set texture for the highway
terrain.SetTexture(veh.GetDataFile("terrain/textures/track.jpg"))
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(2, 0, 0.5), 6.0, 0.5)  # Adjusted camera position
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 5.0  # increased from 1.0 to 5 seconds
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Reference speed input (desired speed)
desired_speed = 15.0  # m/s, set as needed

# PID controller parameters for throttle
Kp = 0.5
Ki = 0.1
Kd = 0.05

integral_error = 0.0
previous_error = 0.0

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between rendering
render_steps = math.ceil(render_step_size / step_size)

# Initialize realtime timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified interval
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # --- PID control for throttle based on speed error ---
    # Obtain current vehicle speed along the x-axis (forward)
    chassis = vehicle.GetVehicle()
    current_speed = chassis.GetSpeed()
    speed_error = desired_speed - current_speed

    # PID calculations
    integral_error += speed_error * step_size
    derivative_error = (speed_error - previous_error) / step_size
    previous_error = speed_error

    # Compute throttle command
    throttle_cmd = Kp * speed_error + Ki * integral_error + Kd * derivative_error
    # Clamp throttle command between -1 and 1
    throttle_cmd = max(min(throttle_cmd, 1.0), -1.0)

    # Set throttle input (assuming driver interface has SetThrottle method)
    driver.SetThrottle(throttle_cmd)

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

    step_number += 1