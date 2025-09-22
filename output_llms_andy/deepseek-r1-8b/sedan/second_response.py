import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation for first vehicle (Sedan)
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial vehicle location and orientation for second vehicle (Truck)
truck_initLoc = chrono.ChVector3d(-10, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
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

# Create the Truck vehicle, set parameters, and initialize
truck = veh.Truck()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.SetTireType(tire_model)
truck.SetTireStepSize(tire_step_size)
truck.Initialize()

# Set visualization types for both vehicles
truck.SetChassisVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

# Create the vehicle Irrlicht interface for both vehicles
vis_sedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()

# Set common visualization parameters
vis_sedan.SetWindowTitle('Sedan')
vis_truck.SetWindowTitle('Truck')
vis_sedan.SetWindowSize(1280, 1024)
vis_truck.SetWindowSize(1280, 1024)

# Set camera positions for both vehicles
sedan_trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)
truck_trackPoint = chrono.ChVector3d(-15.0, 0.0, 1.8)

vis_sedan.SetChaseCamera(sedan_trackPoint, 6.0, 0.5)
vis_truck.SetChaseCamera(truck_trackPoint, 6.0, 0.5)

# Initialize visualization systems
vis_sedan.Initialize()
vis_sedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_sedan.AddLightDirectional()
vis_sedan.AddSkyBox()
vis_sedan.AttachVehicle(vehicle.GetVehicle())

vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(truck.GetVehicle())

# Create and initialize driver systems for both vehicles
driver_sedan = veh.ChInteractiveDriverIRR(vis_sedan)
driver_truck = veh.ChInteractiveDriverIRR(vis_truck)

# Set time response for steering and throttle inputs
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

# Set input deltas
driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)

driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

# Initialize driver systems
driver_sedan.Initialize()
driver_truck.Initialize()

# Create and initialize terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Create terrain for Sedan (shared between both vehicles)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add sinusoidal steering input function
def sinusoidal_input(input_value, frequency=0.1):
    return input_value + math.sin(input_value * frequency)

# Simulation loop
output_vehicle_mass = []
render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis_sedan.Run() and vis_truck.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis_sedan.BeginScene()
        vis_sedan.Render()
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_sedan.EndScene()
        vis_truck.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver_sedan.GetInputs()
    truck_inputs = driver_truck.GetInputs()

    # Synchronize modules
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    truck.Synchronize(time, truck_inputs, terrain)
    vis_sedan.Synchronize(time, driver_inputs)
    vis_truck.Synchronize(time, truck_inputs)

    # Advance simulation
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    truck.Advance(step_size)
    driver_truck.Advance(step_size)

    # Apply sinusoidal steering and throttle inputs
    current_time = time.GetChTime() / step_size
    steering_input = math.sin(current_time * 2)  # Frequency of 1Hz
    throttle_input = math.sin(current_time * 1.5)  # Frequency of 0.66Hz

    # Apply inputs to both vehicles
    driver_sedan.SetSteeringInput(steering_input * 1)
    driver_sedan.SetThrottleInput(throttle_input * 1)
    driver_truck.SetSteeringInput(steering_input * 1)
    driver_truck.SetThrottleInput(throttle_input * 1)

    # Update frame counter
    step_number += 1

    # Spin in place for real time synchronization
    realtime_timer.Spin(step_size)

# Output vehicle mass (for both vehicles)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print("TRUCK MASS: ", truck.GetVehicle().GetMass())