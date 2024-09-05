import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
kraz_initLoc = chrono.ChVector3d(0, 0, 0.5)
kraz_initRot = chrono.ChQuaterniond(1, 0, 0, 0)
sedan_initLoc = chrono.ChVector3d(5, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
kraz_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY

# Predefined highway mesh
terrain_mesh = veh.GetDataFile("terrain/meshes/highway.obj")

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle, set parameters, and initialize
kraz = veh.Kraz()
kraz.SetContactMethod(contact_method)
kraz.SetChassisCollisionType(chassis_collision_type)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(kraz_initLoc, kraz_initRot))
kraz.Initialize()

kraz.SetChassisVisualizationType(vis_type, vis_type)
kraz.SetSteeringVisualizationType(vis_type)
kraz.SetSuspensionVisualizationType(vis_type, vis_type)
kraz.SetWheelVisualizationType(vis_type, vis_type)
kraz.SetTireVisualizationType(vis_type, vis_type)

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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(kraz.GetSystem())
patch = terrain.AddPatch(patch_mat, terrain_mesh)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(kraz_initLoc, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(kraz.GetTractor())
vis.AttachVehicle(sedan.GetTractor())

# Create the drivers
kraz_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChPathFollowerDriver(sedan.GetVehicle(), "my_path.txt", "my_path_pts.txt", 1.0)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
kraz_driver.SetSteeringDelta(render_step_size / steering_time)
kraz_driver.SetThrottleDelta(render_step_size / throttle_time)
kraz_driver.SetBrakingDelta(render_step_size / braking_time)

kraz_driver.Initialize()
sedan_driver.Initialize()

# output vehicle mass
print("KRAZ MASS: ", kraz.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetTractor().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Store the state of the truck's tractor and trailer during the simulation
tractor_states = []
trailer_states = []

while vis.Run():
    time = kraz.GetSystem().GetChTime()

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
    kraz.Synchronize(time, kraz_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, kraz_driver_inputs)

    # Advance simulation for one timestep for all modules
    kraz_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    kraz.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store the state of the truck's tractor and trailer
    tractor_states.append(kraz.GetTractor().GetPos())
    trailer_states.append(kraz.GetTrailer().GetPos())

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)