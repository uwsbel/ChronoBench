import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle initialization parameters
initLoc = chrono.ChVector3d(-40, 0, 0.5)  # Modified initial position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_PRIMITIVES  # Changed for better collision

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation settings
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create vehicle
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Modified terrain texture
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Double Lane Change')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
try:
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())
except Exception as e:
    print(f"Visualization initialization failed: {e}")
    exit()

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Print vehicle mass
vehicle_mass = vehicle.GetVehicle().GetMass()
print(f"VEHICLE MASS: {vehicle_mass}")

# Simulation control variables
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
simulation_time = 0
max_simulation_time = 10  # Simulation duration

# Double lane change maneuver parameters
maneuver_profile = [
    (0, 0, 0.5, 0),  # (time, steering, throttle, braking)
    (1, 0.3, 0.5, 0),  # Start right lane change
    (2, 0, 0.5, 0),    # Straighten
    (3, -0.3, 0.5, 0), # Start left lane change
    (4, 0, 0.2, 0.5)   # Braking and straight
]

current_maneuver_index = 0

# Main simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run() and simulation_time < max_simulation_time:
    time = vehicle.GetSystem().GetChTime()
    simulation_time = time
    
    # Update driver inputs based on maneuver profile
    while current_maneuver_index < len(maneuver_profile) and time >= maneuver_profile[current_maneuver_index][0]:
        steering, throttle, braking = maneuver_profile[current_maneuver_index][1], maneuver_profile[current_maneuver_index][2], maneuver_profile[current_maneuver_index][3]
        driver_inputs = veh.ChDriverInputs()
        driver_inputs.m_steering = steering
        driver_inputs.m_throttle = throttle
        driver_inputs.m_braking = braking
        current_maneuver_index += 1
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Update counters
    step_number += 1
    realtime_timer.Spin(step_size)

# Stop simulation
print("Simulation completed successfully")
vis.Close()