import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(0, 0, 0)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint_truck = chrono.ChVector3d(0, 0, 2.1)
trackPoint_sedan = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the truck vehicle, set parameters, and initialize
vehicle_truck = veh.Kraz()
vehicle_truck.SetContactMethod(contact_method)
vehicle_truck.SetChassisCollisionType(chassis_collision_type)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type, vis_type)
vehicle_truck.SetSteeringVisualizationType(vis_type)
vehicle_truck.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_truck.SetWheelVisualizationType(vis_type, vis_type)
vehicle_truck.SetTireVisualizationType(vis_type, vis_type)

vehicle_truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

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

# Create the sedan vehicle, set parameters, and initialize
vehicle_sedan = veh.Sedan()
vehicle_sedan.SetContactMethod(contact_method)
vehicle_sedan.SetChassisCollisionType(chassis_collision_type)
vehicle_sedan.SetChassisFixed(False)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
vehicle_sedan.Initialize()

vehicle_sedan.SetChassisVisualizationType(vis_type, vis_type)
vehicle_sedan.SetSteeringVisualizationType(vis_type)
vehicle_sedan.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_sedan.SetWheelVisualizationType(vis_type, vis_type)
vehicle_sedan.SetTireVisualizationType(vis_type, vis_type)

vehicle_sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the driver system for the truck
driver_truck = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()

# Create the driver system for the sedan
driver_sedan = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)

driver_sedan.Initialize()

# Store the state of the truck's tractor and trailer during the simulation
vehicle_truck_state = vehicle_truck.GetTractor().GetState()
vehicle_trailer_state = vehicle_truck.GetTrailer().GetState()

# Control the sedan to move forward with fixed throttle and steering
driver_sedan.SetThrottle(1.0)
driver_sedan.SetSteering(0.0)

# Create the simulation
while vis.Run():
    time = vehicle_truck.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver_truck.GetInputs()
    driver_inputs = driver_sedan.GetInputs()

    # Update modules (process inputs from other modules)
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    vehicle_truck.Synchronize(time, driver_inputs, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    vehicle_truck.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)