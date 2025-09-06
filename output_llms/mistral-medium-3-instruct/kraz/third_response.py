import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation

sedan_initLoc = chrono.ChVector3d(5, 0, 0.5)      # 5 meters behind the truck
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
truck_tire_model = veh.TireModelType_RIGID  # Changed to RIGID for truck
sedan_tire_model = veh.TireModelType_TMEASY

# Rigid terrain - using predefined highway mesh
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle (truck), set parameters, and initialize
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

# Create the sedan vehicle, set parameters, and initialize
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

# Set tire models
truck.SetTireTypeFront(truck_tire_model)
truck.SetTireTypeRear(truck_tire_model)
sedan.SetTireTypeFront(sedan_tire_model)
sedan.SetTireTypeRear(sedan_tire_model)

# Create the terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Using predefined highway mesh instead of simple patch
terrain = veh.RigidTerrain(vehicle.GetSystem())
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
terrain.AddVisualizationMesh(highway_mesh, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                           chrono.ChVector2d(terrainLength, terrainWidth))
terrain.AddContactMesh(highway_mesh, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                      chrono.ChVector2d(terrainLength, terrainWidth))

terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan.GetVehicle())

# Create the driver systems
truck_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChDataDriver()

# Set the time response for steering and throttle keyboard inputs for truck
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.Initialize()

# Set fixed throttle and steering for sedan
sedan_throttle = 0.5  # Fixed throttle
sedan_steering = 0.0  # Straight steering
sedan_driver.SetSteering(sedan_steering)
sedan_driver.SetThrottle(sedan_throttle)
sedan_driver.SetBraking(0.0)
sedan_driver.Initialize()

# Output vehicle masses
print("TRUCK MASS: ", truck.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Storage for truck state
truck_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs for truck
    truck_driver_inputs = truck_driver.GetInputs()

    # Fixed inputs for sedan
    sedan_driver_inputs = veh.ChDriverInputs()
    sedan_driver_inputs.m_throttle = sedan_throttle
    sedan_driver_inputs.m_steering = sedan_steering

    # Update modules (process inputs from other modules)
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, truck_driver_inputs)

    # Advance simulation for one timestep for all modules
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store truck state (tractor and trailer positions)
    tractor_pos = truck.GetTractor().GetPos()
    trailer_pos = truck.GetTrailer().GetPos() if truck.GetTrailer() else None
    truck_states.append({
        'time': time,
        'tractor_pos': (tractor_pos.x, tractor_pos.y, tractor_pos.z),
        'trailer_pos': (trailer_pos.x, trailer_pos.y, trailer_pos.z) if trailer_pos else None
    })

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)