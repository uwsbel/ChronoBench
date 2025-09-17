import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create system
contact_method = chrono.ChContactMethod_NSC
system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Initial locations and orientations
initLoc_truck = chrono.ChVector3d(0, 5, 0.5)  # Changed truck location
initLoc_sedan = chrono.ChVector3d(0, -5, 0.5)  # Added sedan location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(0, 0, 2.1)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize truck
vehicle_truck = veh.Kraz(system)
vehicle_truck.SetChassisCollisionType(chassis_collision_type)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot))
vehicle_truck.SetTireType(veh.TireModelType_RIGID)  # Changed to rigid tires
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type, vis_type)
vehicle_truck.SetSteeringVisualizationType(vis_type)
vehicle_truck.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_truck.SetWheelVisualizationType(vis_type, vis_type)
vehicle_truck.SetTireVisualizationType(vis_type, vis_type)
vehicle_truck.SetTireStepSize(tire_step_size)

# Create and initialize sedan
vehicle_sedan = veh.Sedan(system)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot))
vehicle_sedan.Initialize()

vehicle_sedan.SetChassisVisualizationType(vis_type)
vehicle_sedan.SetSuspensionVisualizationType(vis_type)
vehicle_sedan.SetSteeringVisualizationType(vis_type)
vehicle_sedan.SetWheelVisualizationType(vis_type)
vehicle_sedan.SetTireVisualizationType(vis_type)
vehicle_sedan.SetTireStepSize(tire_step_size)

# Create highway terrain mesh
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")  # Highway mesh
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), mesh_file)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle_truck.GetTractor())

# Create drivers
driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChDriver(vehicle_sedan.GetVehicle())
driver_sedan.SetThrottle(0.5)  # Fixed throttle
driver_sedan.SetSteering(0.0)  # Fixed steering

# Set driver response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()

# Output vehicle masses
print("TRUCK MASS: ", vehicle_truck.GetTractor().GetMass())
print("SEDAN MASS: ", vehicle_sedan.GetVehicle().GetMass())

# State logging setup
tractor_states = []
trailer_states = []
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = system.GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

        # Store truck states at render frequency
        tractor = vehicle_truck.GetTractor()
        trailer = vehicle_truck.GetTrailer()
        
        tractor_states.append({
            'time': time,
            'pos': tractor.GetChassisBody().GetPos(),
            'rot': tractor.GetChassisBody().GetRot(),
            'vel': tractor.GetChassisBody().GetPosDt(),
            'ang_vel': tractor.GetChassisBody().GetAngVelLocal()
        })
        
        trailer_states.append({
            'time': time,
            'pos': trailer.GetChassisBody().GetPos(),
            'rot': trailer.GetChassisBody().GetRot(),
            'vel': trailer.GetChassisBody().GetPosDt(),
            'ang_vel': trailer.GetChassisBody().GetAngVelLocal()
        })

    # Get driver inputs
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    # Update modules
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle_truck.Synchronize(time, driver_inputs_truck, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    # Advance simulation
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle_truck.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    # Increment counter
    step_number += 1
    realtime_timer.Spin(step_size)

# After simulation, states are available in:
# tractor_states and trailer_states