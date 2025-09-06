import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Correct data path setup
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle')

# Initial vehicle location and orientation (modified)
initLoc = chrono.ChVector3d(-15, 0, 0.5)  # Changed from (0,0,0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point (modified)
trackPoint = chrono.ChVector3d(3, 0, 2.1)  # Changed from (0,0,2.1)

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize vehicle
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)  # Added missing tire initialization
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat, 
    chrono.CSYSNORM, 
    terrainLength, 
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)  # Modified chase distance
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Configure driver inputs (added missing braking config)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Simulation setup
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Double lane change maneuver parameters
maneuver_start_time = 2.0
maneuver_duration = 8.0
phase_duration = maneuver_duration / 4

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Double lane change maneuver sequence
    if time > maneuver_start_time and time < maneuver_start_time + maneuver_duration:
        maneuver_time = time - maneuver_start_time
        phase = int(maneuver_time / phase_duration)
        
        if phase == 0:  # First turn right
            driver.SetSteering(0.5)
            driver.SetThrottle(0.7)
        elif phase == 1:  # Turn left
            driver.SetSteering(-0.7)
            driver.SetThrottle(0.7)
        elif phase == 2:  # Turn right again
            driver.SetSteering(0.7)
            driver.SetThrottle(0.7)
        elif phase == 3:  # Straighten out
            driver.SetSteering(-0.3)
            driver.SetThrottle(0.7)
    else:
        # Default driving behavior
        driver.SetThrottle(0.5)
        driver.SetSteering(0.0)

    # Rendering
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get and process driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment counters
    step_number += 1
    realtime_timer.Spin(step_size)