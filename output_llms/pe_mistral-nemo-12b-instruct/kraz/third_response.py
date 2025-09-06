import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(10, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization types for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type_truck = veh.VisualizationType_MESH
vis_type_sedan = veh.VisualizationType_MESH

# Collision types for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type_truck = veh.CollisionType_NONE
chassis_collision_type_sedan = veh.CollisionType_MESH

# Types of tire models (RIGID, TMEASY)
tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY

# Rigid terrain
terrain_model = veh.RigidTerrain.MESH
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Points tracked by the cameras
trackPoint_truck = chrono.ChVector3d(0, 0, 2.1)
trackPoint_sedan = chrono.ChVector3d(10, 0, 2.1)

# Contact methods
contact_method_truck = chrono.ChContactMethod_NSC
contact_method_sedan = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the truck vehicle, set parameters, and initialize
vehicle_truck = veh.Kraz()
vehicle_truck.SetContactMethod(contact_method_truck)
vehicle_truck.SetChassisCollisionType(chassis_collision_type_truck)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type_truck, vis_type_truck)
vehicle_truck.SetSteeringVisualizationType(vis_type_truck)
vehicle_truck.SetSuspensionVisualizationType(vis_type_truck, vis_type_truck)
vehicle_truck.SetWheelVisualizationType(vis_type_truck, vis_type_truck)
vehicle_truck.SetTireVisualizationType(vis_type_truck, vis_type_truck)

vehicle_truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the sedan vehicle, set parameters, and initialize
vehicle_sedan = veh.Sedan()
vehicle_sedan.SetContactMethod(contact_method_sedan)
vehicle_sedan.SetChassisCollisionType(chassis_collision_type_sedan)
vehicle_sedan.SetChassisFixed(False)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
vehicle_sedan.Initialize()

vehicle_sedan.SetChassisVisualizationType(vis_type_sedan, vis_type_sedan)
vehicle_sedan.SetSteeringVisualizationType(vis_type_sedan)
vehicle_sedan.SetSuspensionVisualizationType(vis_type_sedan, vis_type_sedan)
vehicle_sedan.SetWheelVisualizationType(vis_type_sedan, vis_type_sedan)
vehicle_sedan.SetTireVisualizationType(vis_type_sedan, vis_type_sedan)

vehicle_sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle_truck.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interfaces
vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck.SetWindowTitle('Truck Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(trackPoint_truck, 25.0, 1.5)
vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(vehicle_truck.GetTractor())

vis_sedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_sedan.SetWindowTitle('Sedan Demo')
vis_sedan.SetWindowSize(1280, 1024)
vis_sedan.SetChaseCamera(trackPoint_sedan, 25.0, 1.5)
vis_sedan.Initialize()
vis_sedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_sedan.AddLightDirectional()
vis_sedan.AddSkyBox()
vis_sedan.AttachVehicle(vehicle_sedan.GetTractor())

# Create the driver systems
driver_truck = veh.ChInteractiveDriverIRR(vis_truck)
driver_sedan = veh.ChInteractiveDriverIRR(vis_sedan)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()
driver_sedan.Initialize()

# Output vehicle masses
print("TRUCK MASS: ", vehicle_truck.GetTractor().GetMass())
print("SEDAN MASS: ", vehicle_sedan.GetTractor().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Initialize sedan throttle and steering
throttle_sedan = 0.5
steering_sedan = 0.0

while vis_truck.Run() and vis_sedan.Run():
    time = vehicle_truck.GetSystem().GetChTime()

    # Render scenes and output POV-Ray data
    if (step_number % render_steps == 0):
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        vis_sedan.BeginScene()
        vis_sedan.Render()
        vis_sedan.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Update modules (process inputs from other modules)
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time, throttle=throttle_sedan, steering=steering_sedan)
    terrain.Synchronize(time)
    vehicle_truck.Synchronize(time, driver_inputs_truck, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis_truck.Synchronize(time, driver_inputs_truck)
    vis_sedan.Synchronize(time, driver_inputs_sedan)

    # Advance simulation for one timestep for all modules
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle_truck.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis_truck.Advance(step_size)
    vis_sedan.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Control sedan to move forward with fixed throttle and steering
    if step_number % 100 == 0:
        throttle_sedan = 0.5
        steering_sedan = 0.0

    # Store truck's tractor and trailer state
    if step_number % 1000 == 0:
        truck_state = vehicle_truck.GetTractor().GetPos()
        trailer_state = vehicle_truck.GetTrailer().GetPos()
        print(f"Truck state at step {step_number}: Tractor={truck_state}, Trailer={trailer_state}")