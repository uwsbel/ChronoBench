import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
truck_initLoc = chrono.ChVector3d(5, 0, 0.5)  # Changed initial position for truck
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

sedan_initLoc = chrono.ChVector3d(-5, 0, 0.5)  # New position for sedan
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type for truck (RIGID, TMEASY)
truck_tire_model = veh.TireModelType_RIGID  # Changed to RIGID for truck

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize the truck
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

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and initialize the sedan
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

# Create the terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())  # Use truck's system for terrain

# Load highway mesh
mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(veh.GetDataFile('terrain/meshes/highway.obj'), False, False)
patch = terrain.AddPatchFromMesh(patch_mat, mesh, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck.GetChassis().GetPoint(truck.GetChassis().GetPointNames()[0]), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

# Add both vehicles to visualization
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan)

# Create driver systems
truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()

sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.8)  # Fixed throttle for sedan
sedan_driver.SetSteering(0.0)  # Fixed steering for sedan

# Initialize simulation variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Lists to store vehicle states
truck_tractor_states = []
truck_trailer_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Store truck states
    if step_number % render_steps == 0:
        tractor_pos = truck.GetTractor().GetPos()
        trailer_pos = truck.GetTrailer().GetPos()
        truck_tractor_states.append((tractor_pos.x, tractor_pos.y, tractor_pos.z))
        truck_trailer_states.append((trailer_pos.x, trailer_pos.y, trailer_pos.z))

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    truck_inputs = truck_driver.GetInputs()
    sedan_inputs = sedan_driver.GetInputs()

    # Update modules
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)

    # Advance simulation
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time
    realtime_timer.Spin(step_size)