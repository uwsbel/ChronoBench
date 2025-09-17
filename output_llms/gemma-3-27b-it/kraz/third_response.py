import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle (Kraz) location and orientation
kraz_initLoc = chrono.ChVector3d(-10, 0, 0.5)
kraz_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Initial sedan location and orientation
sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
kraz_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Kraz vehicle, set parameters, and initialize
kraz_vehicle = veh.Kraz()
kraz_vehicle.SetContactMethod(contact_method)
kraz_vehicle.SetChassisCollisionType(chassis_collision_type)
kraz_vehicle.SetChassisFixed(False)
kraz_vehicle.SetInitPosition(chrono.ChCoordsysd(kraz_initLoc, kraz_initRot))
kraz_vehicle.Initialize()
kraz_vehicle.SetChassisVisualizationType(vis_type, vis_type)
kraz_vehicle.SetSteeringVisualizationType(vis_type)
kraz_vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
kraz_vehicle.SetWheelVisualizationType(vis_type, vis_type)
kraz_vehicle.SetTireVisualizationType(vis_type, vis_type)
kraz_vehicle.SetTireModel(kraz_tire_model)
kraz_vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the sedan vehicle, set parameters, and initialize
sedan_vehicle = veh.Sedan()
sedan_vehicle.SetContactMethod(contact_method)
sedan_vehicle.SetChassisCollisionType(chassis_collision_type)
sedan_vehicle.SetChassisFixed(False)
sedan_vehicle.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan_vehicle.Initialize()
sedan_vehicle.SetChassisVisualizationType(vis_type, vis_type)
sedan_vehicle.SetSteeringVisualizationType(vis_type)
sedan_vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
sedan_vehicle.SetWheelVisualizationType(vis_type, vis_type)
sedan_vehicle.SetTireVisualizationType(vis_type, vis_type)
sedan_vehicle.SetTireModel(sedan_tire_model)
sedan_vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


# Create the terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Use a predefined highway mesh
terrain = veh.HighwayTerrain(kraz_vehicle.GetSystem())
terrain.Initialize()
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(kraz_vehicle.GetTractor())
vis.AttachVehicle(sedan_vehicle.GetTractor())

# Create the driver system for Kraz
kraz_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

kraz_driver.SetSteeringDelta(render_step_size / steering_time)
kraz_driver.SetThrottleDelta(render_step_size / throttle_time)
kraz_driver.SetBrakingDelta(render_step_size / braking_time)
kraz_driver.Initialize()

# Control the sedan to move forward with fixed throttle and steering
sedan_driver.SetThrottle(1.0)
sedan_driver.SetSteering(0.0)
sedan_driver.Initialize()

# Store the state of the truck's tractor and trailer
tractor = kraz_vehicle.GetTractor()
trailer = kraz_vehicle.GetTrailer()

# output vehicle mass
print("VEHICLE MASS (Kraz): ", kraz_vehicle.GetTractor().GetMass())
print("VEHICLE MASS (Sedan): ", sedan_vehicle.GetTractor().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
s = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = kraz_vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    kraz_driver_inputs = kraz_driver.GetInputs()
    sedan_driver_inputs = sedan_driver.GetInputs()

    # Update modules (process inputs from other modules)
    kraz_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    kraz_vehicle.Synchronize(time, kraz_driver_inputs, terrain)
    sedan_vehicle.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, kraz_driver_inputs)
    vis.Synchronize(time, sedan_driver_inputs)

    # Advance simulation for one timestep for all modules
    kraz_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    kraz_vehicle.Advance(step_size)
    sedan_vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    s.Spin(step_size)