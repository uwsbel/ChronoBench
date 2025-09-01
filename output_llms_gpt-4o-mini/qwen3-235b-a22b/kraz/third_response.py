import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
# Truck initial location and orientation (moved to Y=2, rotated 180 degrees)
initLoc = chrono.ChVector3d(0, 2, 0.5)
initRot = chrono.Q_from_AngY(math.pi)

# Sedan initial location and orientation
sedanInitLoc = chrono.ChVector3d(0, -2, 0.5)
sedanInitRot = chrono.Q_from_AngY(0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID  # Changed to RIGID

# Terrain
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point tracked by camera (now on truck's tractor)
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50

# Create and initialize the truck
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.SetTireType(tire_model)  # Apply tire model
truck.SetTireStepSize(tire_step_size)
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
sedan.SetInitPosition(chrono.ChCoordsysd(sedanInitLoc, sedanInitRot))
sedan.SetTireType(tire_model)  # Assuming same tire model
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

# Create the terrain with a highway mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())

# Load highway mesh (example path; adjust as needed)
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    highway_mesh,
    0.01)  # Assuming thickness 0.01

patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))  # Set color for mesh
terrain.Initialize()

# Create the visualization system and add both vehicles
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicles Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())  # Attach truck's tractor
vis.AttachVehicle(sedan.GetVehicle())  # Attach sedan

# Create the truck's driver system
truck_driver = veh.ChInteractiveDriverIRR(vis)
# Set truck's driver time response
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.Initialize()

# Create the sedan's driver system (non-interactive, fixed throttle/steering)
sedan_driver = veh.ChDriver(sedan.GetVehicle())
# Set time response for completeness (though inputs will be overridden)
sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)
sedan_driver.Initialize()

# Output vehicle masses
print("TRUCK MASS:", truck.GetTractor().GetMass())
print("SEDAN MASS:", sedan.GetVehicle().GetMass())

# State storage
truck_states = []
trailer_states = []

# Number of steps between renders
render_steps = math.ceil(render_step_size / step_size)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = truck.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get truck driver inputs
    truck_inputs = truck_driver.GetInputs()

    # Update all modules
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)  # This may set inputs, but we'll override
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    
    # Set sedan inputs to fixed throttle and zero steering
    sedan_inputs = sedan_driver.GetInputs()
    sedan_inputs.m_throttle = 0.7
    sedan_inputs.m_steering = 0.0
    sedan.Synchronize(time, sedan_inputs, terrain)
    
    vis.Synchronize(time, truck_inputs)

    # Advance all modules
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # Store truck and trailer states
    tractor = truck.GetTractor()
    trailer = truck.GetTrailer()
    truck_states.append({
        'time': time,
        'pos': tractor.GetPos(),
        'rot': tractor.GetRot(),
        'vel': tractor.GetPos_dt()
    })
    if trailer:
        trailer_states.append({
            'time': time,
            'pos': trailer.GetPos(),
            'rot': trailer.GetRot(),
            'vel': trailer.GetPos_dt()
        })

    # Increment step
    step_number += 1
    realtime_timer.Spin(step_size)