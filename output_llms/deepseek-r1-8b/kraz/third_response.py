import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial truck location and orientation
initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial sedan location and orientation
initLoc_sedan = chrono.ChVector3d(-50, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type for the truck (RIGID, TMEASY)
tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_RIGID  # Use same model for sedan

# Rigid terrain
terrain_model = veh.RigidTerrain.Type_HIGHWAY_MESH
terrainHeight = 0      # terrain height
terrainLength = 1000.0  # size in X direction
terrainWidth = 300.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0,0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Kraz truck, set parameters, and initialize
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
terrain = veh.RigidTerrain(truck.GetSystem())
terrain.InitializeTerrain(terrain_model, terrainLength, terrainWidth, terrainHeight)

# Create the truck Irrlicht interface
vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck.SetWindowTitle('Kraz Truck Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(trackPoint, 25.0, 1.5)
vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(truck.GetTractor())

# Create the sedan Irrlicht interface
sedan_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
sedan_vis.SetWindowTitle('Sedan Demo')
sedan_vis.SetWindowSize(1280, 1024)
sedan_vis.SetChaseCamera(trackPoint, 25.0, 1.5)
sedan_vis.Initialize()
sedan_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
sedan_vis.AddLightDirectional()
sedan_vis.AddSkyBox()
sedan_vis.AttachVehicle(sedan.GetTractor())

# Create the first driver system for the truck
driver_truck = veh.ChInteractiveDriverIRR(vis_truck)

# Create the second driver system for the sedan
driver_sedan = veh.ChInteractiveDriverIRR(sedan_vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()

# output vehicle mass
print("VEHICLE MASS: ", truck.GetTractor().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Store truck state during simulation
truck_state = None

while vis_truck.Run() and vis_sedan.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver_truck.GetInputs()

    # Update modules (process inputs from other modules)
    truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Advance(step_size)
    vis_truck.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis_truck.Advance(step_size)

    # Move sedan automatically
    if not truck.GetTractor().GetState().IsBraking():
        sedan_control = sedan.GetTractor().GetControl()
        sedan_control.SetThrottle(0.5)  # Constant forward speed
        sedan_control.SetSteering(0.0)  # Straight movement

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

    # Store truck state
    if truck_state is None:
        truck_state = truck.GetTractor().GetState().Copy()

# Load truck state
if truck_state is not None:
    truck.GetTractor().GetState().Load(truck_state)