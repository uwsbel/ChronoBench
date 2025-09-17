import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

# Set the data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation for BMW
initLoc = chrono.ChVector3d(0, 0, 0.5)
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

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# --------------
# Create systems
# --------------

# Create the first vehicle (BMW)
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Create the second vehicle (Truck)
truck = veh.Truck()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.QUNIT))
truck.SetTireType(tire_model)
truck.SetTireStepSize(tire_step_size)
truck.Initialize()

# Create the vehicle Irrlicht interface for both vehicles
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan & Truck')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())
vis.AttachVehicle(truck.GetVehicle())

# Create the driver systems for both vehicles
driver = veh.ChInteractiveDriverIRR(vis)
truck_driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

# Sinusoidal steering input
def get_steering_input(time):
    return 1.0 * math.sin(time / 10)  # Scale down to avoid excessive input

# Set input deltas for both drivers
driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetSteeringDelta(render_step_size / steering_time)

driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)

driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)

# Initialize both drivers
driver.Initialize()
truck_driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# Output initial vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print("TRUCK MASS: ", truck.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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

    # Synchronize all modules
    vehicle.Synchronize(time)
    truck.Synchronize(time)
    driver.Synchronize(time)
    truck_driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    vehicle.Advance(step_size)
    truck.Advance(step_size)
    driver.Advance(step_size)
    truck_driver.Advance(step_size)

    # Apply sinusoidal steering to both vehicles
    steering_angle = get_steering_input(time)
    if steering_angle > 1.0:
        steering_angle = 1.0
    elif steering_angle < -1.0:
        steering_angle = -1.0
    driver.SetSteeringInput(steering_angle)
    truck_driver.SetSteeringInput(steering_angle)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)