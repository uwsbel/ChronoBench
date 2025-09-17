import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial truck location and orientation
initLocTruck = chrono.ChVector3d(0, 0, 0.5)
initRotTruck = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial sedan location and orientation
initLocSedan = chrono.ChVector3d(0, 0, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type for the truck (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain using predefined highway mesh
terrainHeight = 0      # terrain height
terrainLength = 400.0  # size in X direction for highway
terrainWidth = 20.0   # size in Y direction for highway

# Poon chassis tracked by the camera for truck
trackPointTruck = chrono.ChVector3d(0,0, 2.1)

# Poon chassis tracked by the camera for sedan
trackPointSedan = chrono.ChVector3d(0,0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the truck, set parameters, and initialize
vehicleTruck = veh.Kraz()
vehicleTruck.SetContactMethod(contact_method)
vehicleTruck.SetChassisCollisionType(chassis_collision_type)
vehicleTruck.SetChassisFixed(False)
vehicleTruck.SetInitPosition(chrono.ChCoordsysd(initLocTruck, initRotTruck))
vehicleTruck.Initialize()

# Create the sedan, set parameters, and initialize
vehicleSedan = veh.Sedan()
vehicleSedan.SetContactMethod(contact_method)
vehicleSedan.SetChassisCollisionType(chassis_collision_type)
vehicleSedan.SetChassisFixed(False)
vehicleSedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
vehicleSedan.Initialize()

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicleTruck.GetSystem())
terrain.CreateBoxMesh(chrono.ChVector3d(0, 0, 0), terrainLength, terrainWidth, 1, 1)
terrain.Initialize()

# Create the vehicle Irrlicht interface for truck
visTruck = veh.ChWheeledVehicleVisualSystemIrrlicht()
visTruck.SetWindowTitle('Kraz Demo')
visTruck.SetWindowSize(1280, 1024)
visTruck.SetChaseCamera(trackPointTruck, 25.0, 1.5)
visTruck.Initialize()
visTruck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visTruck.AddLightDirectional()
visTruck.AddSkyBox()
visTruck.AttachVehicle(vehicleTruck.GetTractor())

# Create the driver system for truck
driverTruck = veh.ChInteractiveDriverIRR(visTruck)

# Create the vehicle Irrlicht interface for sedan
visSedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
visSedan.SetWindowTitle('Sedan Demo')
visSedan.SetWindowSize(1280, 1024)
visSedan.SetChaseCamera(trackPointSedan, 25.0, 1.5)
visSedan.Initialize()
visSedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visSedan.AddLightDirectional()
visSedan.AddSkyBox()
visSedan.AttachVehicle(vehicleSedan.GetTractor())

# Create the second driver system for sedan
driverSedan = veh.ChInteractiveDriverIRR(visSedan)

# Set time response for steering and throttle keyboard inputs for both drivers
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driverTruck.SetSteeringDelta(render_step_size / steering_time)
driverTruck.SetThrottleDelta(render_step_size / throttle_time)
driverTruck.SetBrakingDelta(render_step_size / braking_time)
driverSedan.SetSteeringDelta(render_step_size / steering_time)
driverSedan.SetThrottleDelta(render_step_size / throttle_time)
driverSedan.SetBrakingDelta(render_step_size / braking_time)

# Store truck's tractor and trailer states during simulation
truck_states = []

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while visTruck.Run() and visSedan.Run():
    time = vehicleTruck.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        visTruck.BeginScene()
        visTruck.Render()
        visTruck.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driverTruck.GetInputs()
    driver_inputs_sedan = driverSedan.GetInputs()

    # Update modules (process inputs from other modules)
    driverTruck.Synchronize(time)
    terrain.Synchronize(time)
    vehicleTruck.Synchronize(time, driver_inputs)
    visTruck.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driverTruck.Advance(step_size)
    terrain.Advance(step_size)
    vehicleTruck.Advance(step_size)
    visTruck.Advance(step_size)

    # Get current state of truck's tractor and trailer
    truck_state = vehicleTruck.GetTractor().GetState()
    trailer_state = vehicleTruck.GetTrailer().GetState()
    truck_states.append((truck_state, trailer_state))

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Update sedan driver inputs and advance simulation
    driverSedan.Synchronize(time, driver_inputs_sedan)
    vehicleSedan.Synchronize(time, driver_inputs_sedan)
    driverSedan.Advance(step_size)
    visSedan.Advance(step_size)

    # Print truck state every 100 frames for visualization
    if step_number % 100 == 0:
        print(f"TRUCK STATE: {truck_states[-1][0]}")
        print(f"TRUCK TRAILER STATE: {truck_states[-1][1]}")