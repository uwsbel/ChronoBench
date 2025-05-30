import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np  # Added for data storage

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Changed initial truck location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)  # Changed height from 0.5 to 1.0
initRot = chrono.ChQuaterniond(chrono.Q_from_AngY(math.pi/6))  # Changed rotation to turn 30 degrees

# Added initial location and orientation for a sedan
sedanLoc = chrono.ChVector3d(10, -5, 0.5)
sedanRot = chrono.ChQuaterniond(chrono.Q_from_AngY(-math.pi/4))  # -45 degrees rotation

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Changed tire model type for the truck to rigid
tire_model = veh.TireModelType_RIGID  # Changed from TMEASY to RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Points tracked by the cameras
trackPoint = chrono.ChVector3d(0, 0, 2.1)
sedanTrackPoint = chrono.ChVector3d(0, 0, 1.5)  # Track point for sedan

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
vehicle.SetTireType(tire_model)  # Setting tire model to RIGID
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create and initialize the sedan (second vehicle)
sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedanLoc, sedanRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Set the collision system for both vehicles
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Updated terrain to use a predefined highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Use a predefined highway mesh instead of a flat terrain
highway_file = veh.GetDataFile("terrain/meshes/highway.obj")  # Assuming this file exists
patch = terrain.AddPatch(patch_mat, highway_file, "highway_mesh", 0.01, False)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

# Create the driver system for the truck
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Create the driver system for the sedan with fixed controls
sedan_driver = veh.ChDriver(sedan)
# Controlled the sedan to move forward with fixed throttle and steering
sedan_throttle = 0.5  # 50% throttle
sedan_steering = 0.1  # slight right turn
sedan_driver_inputs = veh.ChDriverInputs()
sedan_driver_inputs.m_throttle = sedan_throttle
sedan_driver_inputs.m_steering = sedan_steering
sedan_driver_inputs.m_braking = 0.0

# output vehicle masses
print("TRUCK MASS: ", vehicle.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Stored the state of the truck's tractor and trailer during the simulation
max_frames = 1000
truck_tractor_positions = np.zeros((max_frames, 3))
truck_trailer_positions = np.zeros((max_frames, 3))
frame_times = np.zeros(max_frames)
current_frame = 0

while vis.Run() and current_frame < max_frames:
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs for the truck
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    sedan_driver.Synchronize(time, sedan_driver_inputs)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store truck tractor and trailer positions
    if step_number % render_steps == 0 and current_frame < max_frames:
        tractor_pos = vehicle.GetTractor().GetPos()
        trailer_pos = vehicle.GetTrailer().GetPos()
        truck_tractor_positions[current_frame] = [tractor_pos.x, tractor_pos.y, tractor_pos.z]
        truck_trailer_positions[current_frame] = [trailer_pos.x, trailer_pos.y, trailer_pos.z]
        frame_times[current_frame] = time
        current_frame += 1

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# Save the stored positions to file (optional)
np.savez('truck_trajectory.npz', 
         tractor_positions=truck_tractor_positions[:current_frame], 
         trailer_positions=truck_trailer_positions[:current_frame],
         times=frame_times[:current_frame])