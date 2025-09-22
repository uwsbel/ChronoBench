import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Initial vehicle locations and orientations ---

# Truck initial location and orientation
truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Sedan initial location and orientation
sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type for the truck (changed to RIGID)
tire_model_truck = veh.TireModelType_RIGID

# Terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Track point for camera
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # FPS = 50

# --- Create the truck vehicle ---
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

# Create terrain with highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(truck.GetSystem())

# Load highway mesh (assuming a predefined mesh file)
highway_mesh_file = 'highway.obj'  # Replace with actual file path if available
patch = terrain.AddMesh(chrono.GetChronoDataFile(highway_mesh_file),
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT))

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface for truck
vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck.SetWindowTitle('Kraz Truck Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(trackPoint, 25.0, 1.5)
vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(truck.GetTractor())

# Create driver system for truck
driver_truck = veh.ChInteractiveDriverIRR(vis_truck)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()

print("VEHICLE MASS (truck):", truck.GetTractor().GetMass())

# --- Create the sedan vehicle ---
sedan = veh.Kraz()  # Assuming same vehicle class; replace with actual sedan class if available
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

# For the sedan, change tire model to RIGID
sedan.SetTireModelType(tire_model_truck)

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

# Create the vehicle Irrlicht interface for sedan
vis_sedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_sedan.SetWindowTitle('Sedan Demo')
vis_sedan.SetWindowSize(1280, 1024)
vis_sedan.SetChaseCamera(chrono.ChVector3d(10, 0, 2.1), 25.0, 1.5)
vis_sedan.Initialize()
vis_sedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_sedan.AddLightDirectional()
vis_sedan.AddSkyBox()
vis_sedan.AttachVehicle(sedan.GetTractor())

# Create driver system for sedan
driver_sedan = veh.ChInteractiveDriverIRR(vis_sedan)
driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)
driver_sedan.Initialize()

print("VEHICLE MASS (sedan):", sedan.GetTractor().GetMass())

# --- Simulation loop ---
import math

# Number of steps between rendering
render_steps = math.ceil(render_step_size / step_size)

# Timer for real-time simulation
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

# Variables to store tractor and trailer states for the truck
tractor_state = None
trailer_state = None

while vis_truck.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene at specified intervals
    if (step_number % render_steps == 0):
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        render_frame += 1

    # Get driver inputs for truck
    driver_inputs_truck = driver_truck.GetInputs()

    # Synchronize modules
    driver_truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    vis_truck.Synchronize(time, driver_inputs_truck)

    # Advance modules
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis_truck.Advance(step_size)

    # Store tractor and trailer states at certain intervals or conditions
    if step_number % render_steps == 0:
        # Save the current state of the tractor and trailer
        tractor_state = {
            'pos': truck.GetTractor().GetPos(),
            'rot': truck.GetTractor().GetRot(),
            'vel': truck.GetTractor().GetPos_dt(),
            'ang_vel': truck.GetTractor().GetWvel_par()
        }
        # If trailer exists, store its state similarly
        trailer = truck.GetTrailer()
        if trailer:
            trailer_state = {
                'pos': trailer.GetPos(),
                'rot': trailer.GetRot(),
                'vel': trailer.GetPos_dt(),
                'ang_vel': trailer.GetWvel_par()
            }

    # For the sedan, move forward with fixed throttle and steering
    # Example: set throttle to 0.8 and steering to 0.0
    driver_sedan.SetInput(veh.DriverInputs(steering=0.0, throttle=0.8, braking=0.0))
    # Synchronize and advance sedan
    driver_sedan.Synchronize(time)
    sedan.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Advance(step_size)
    vis_sedan.Synchronize(time, driver_inputs_truck)
    vis_sedan.BeginScene()
    vis_sedan.Render()
    vis_sedan.EndScene()

    # Increment step number
    step_number += 1

    # Spin in place for real time
    realtime_timer.Spin(step_size)