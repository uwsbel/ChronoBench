import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set correct Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
truck_init_loc = chrono.ChVector3d(0, 0, 0.5)
truck_init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
sedan_init_loc = chrono.ChVector3d(0, 5, 0.5)
sedan_init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model types
truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY

# Highway terrain with mesh
terrain_file = "highway.obj"
terrain_length = 100.0
terrain_width = 100.0

# Camera tracking point
track_point = chrono.ChVector3d(0, 0, 2.1)

# Contact method and visualization
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS

# Create Kraz truck and Sedan vehicles
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_init_loc, truck_init_rot))
truck.SetTireModel(truck_tire_model)
truck.Initialize()

truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)

sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_init_loc, sedan_init_rot))
sedan.SetTireModel(sedan_tire_model)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# Create highway terrain with mesh
terrain = veh.HighwayTerrain(truck.GetSystem())
terrain.LoadMesh(veh.GetDataFile(terrain_file))
terrain.SetFriction(0.9)
terrain.SetRestitution(0.01)
terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Set collision system
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())

# Create driver systems
interactive_driver = veh.ChInteractiveDriverIRR(vis)
interactive_driver.SetSteeringDelta(render_step_size / 1.0)
interactive_driver.SetThrottleDelta(render_step_size / 1.0)
interactive_driver.SetBrakingDelta(render_step_size / 0.3)
interactive_driver.Initialize()

fixed_driver = veh.ChDriver(sedan)
fixed_driver.SetInputs(0.0, 0.5, 0.0)  # Fixed steering, throttle, braking

# Store vehicle states
tractor_states = []
trailer_states = []

# Print vehicle masses
print("TRUCK MASS:", truck.GetTractor().GetMass())
print("SEDAN MASS:", sedan.GetTractor().GetMass())

# Simulation parameters
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

# Realtime timer
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation loop
while vis.Run():
    time = truck.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    truck_inputs = interactive_driver.GetInputs()
    sedan_inputs = fixed_driver.GetInputs()

    # Update vehicle states
    interactive_driver.Synchronize(time)
    fixed_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)

    # Store tractor and trailer states
    tractor_states.append((truck.GetTractor().GetPos(), truck.GetTractor().GetRot()))
    trailer_states.append((truck.GetTrailer().GetPos(), truck.GetTrailer().GetRot()))

    # Advance simulation
    interactive_driver.Advance(step_size)
    fixed_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)