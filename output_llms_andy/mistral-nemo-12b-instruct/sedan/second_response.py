import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

print(veh)

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation for vehicle 1
initLoc1 = chrono.ChVector3d(0, 0, 0.5)
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial vehicle location and orientation for vehicle 2
initLoc2 = chrono.ChVector3d(10, 0, 0.5)
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)

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

# Point tracked by the camera for vehicle 1
trackPoint1 = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Point tracked by the camera for vehicle 2
trackPoint2 = chrono.ChVector3d(5.0, 0.0, 1.8)

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

# Create the first vehicle, set parameters, and initialize
vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)

vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the second vehicle, set parameters, and initialize
vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)

vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle1.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface for vehicle 1
vis1 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis1.SetWindowTitle('Vehicle 1')
vis1.SetWindowSize(1280, 1024)
vis1.SetChaseCamera(trackPoint1, 6.0, 0.5)
vis1.Initialize()
vis1.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis1.AddLightDirectional()
vis1.AddSkyBox()
vis1.AttachVehicle(vehicle1.GetVehicle())

# Create the vehicle Irrlicht interface for vehicle 2
vis2 = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis2.SetWindowTitle('Vehicle 2')
vis2.SetWindowSize(1280, 1024)
vis2.SetChaseCamera(trackPoint2, 6.0, 0.5)
vis2.Initialize()
vis2.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis2.AddLightDirectional()
vis2.AddSkyBox()
vis2.AttachVehicle(vehicle2.GetVehicle())

# Create the driver system for vehicle 1
driver1 = veh.ChInteractiveDriverIRR(vis1)

# Set the time response for steering and throttle keyboard inputs for vehicle 1
steering_time1 = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time1 = 1.0  # time to go from 0 to +1
braking_time1 = 0.3   # time to go from 0 to +1
driver1.SetSteeringDelta(render_step_size / steering_time1)
driver1.SetThrottleDelta(render_step_size / throttle_time1)
driver1.SetBrakingDelta(render_step_size / braking_time1)

driver1.Initialize()

# Create the driver system for vehicle 2
driver2 = veh.ChInteractiveDriverIRR(vis2)

# Set the time response for steering and throttle keyboard inputs for vehicle 2
steering_time2 = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time2 = 1.0  # time to go from 0 to +1
braking_time2 = 0.3   # time to go from 0 to +1
driver2.SetSteeringDelta(render_step_size / steering_time2)
driver2.SetThrottleDelta(render_step_size / throttle_time2)
driver2.SetBrakingDelta(render_step_size / braking_time2)

driver2.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle mass for both vehicles
print( "VEHICLE 1 MASS: ",  vehicle1.GetVehicle().GetMass())
print( "VEHICLE 2 MASS: ",  vehicle2.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis1.Run() and vis2.Run() :
    time = vehicle1.GetSystem().GetChTime()

    # Render scene and output POV-Ray data for both vehicles
    if (step_number % render_steps == 0) :
        vis1.BeginScene()
        vis1.Render()
        vis1.EndScene()
        vis2.BeginScene()
        vis2.Render()
        vis2.EndScene()
        render_frame += 1

    # Get driver inputs for both vehicles
    driver_inputs1 = driver1.GetInputs()
    driver_inputs2 = driver2.GetInputs()

    # Update modules (process inputs from other modules) for both vehicles
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    vehicle1.Synchronize(time, driver_inputs1, terrain)
    vehicle2.Synchronize(time, driver_inputs2, terrain)
    vis1.Synchronize(time, driver_inputs1)
    vis2.Synchronize(time, driver_inputs2)

    # Advance simulation for one timestep for all modules for both vehicles
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis1.Advance(step_size)
    vis2.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)