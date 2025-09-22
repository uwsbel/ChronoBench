import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction (increased for maneuver)
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle, set parameters, and initialize
vehicle = veh.FEDA()
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system (path-follower with cruise control)
path_points = [
    chrono.ChCoordsysd(chrono.ChVector3d(-50, 0, 0.5), chrono.QUNIT),  # Start position
    chrono.ChCoordsysd(chrono.ChVector3d(-50, 50, 0.5), chrono.QUNIT),  # First straight segment
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 50, 0.5), chrono.QUNIT),  # Second straight segment
    chrono.ChCoordsysd(chrono.ChVector3d(0, 50, 0.5), chrono.QUNIT),  # Curve start
    chrono.ChCoordsysd(chrono.ChVector3d(25, 50, 0.5), chrono.QUNIT),  # Curve end
    chrono.ChCoordsysd(chrono.ChVector3d(50, 50, 0.5), chrono.QUNIT)   # End position
]

path_follower = veh.ChPathFollowerDriver(vehicle.GetVehicle())
cruise_control = veh.ChCruiseControlDriver(path_follower, vehicle.GetVehicle().GetSystem())

# Path-follower configuration
path_follower.SetPath(path_points)
path_follower.SetTargetSpeed(10.0)  # Target speed for path following
path_follower.SetLookaheadDistance(5.0)  # Lookahead distance for steering
path_follower.SetSteeringGain(0.1)  # Steering gain for smooth transitions
path_follower.SetSpeedGain(1.0)  # Speed gain for proportional speed control

# Cruise control configuration
cruise_control.SetMaxSpeed(10.0)
cruise_control.SetAccelerationGain(5.0)
cruise_control.SetDecelerationGain(5.0)

driver = veh.ChInteractiveDriverIRR(vis)  # Remove this line in final code

# Set the time response for steering and throttle keyboard inputs (commented out)
# driver.SetSteeringDelta(render_step_size / steering_time)
# driver.SetThrottleDelta(render_step_size / throttle_time)
# driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

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

    # Get driver inputs (no longer used in path-follower system)
    driver_inputs = driver.GetInputs()

    # Update modules with current time
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)