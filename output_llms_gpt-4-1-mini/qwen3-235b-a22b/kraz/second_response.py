import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)
initRot = chrono.ChQuaterniond(0, 0, 1, 0)  # 180-degree rotation about Y-axis

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVector3d(3, 0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the kraz vehicle and configure it
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)  # Apply the tire model type

# Set visualization types before initialization
vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.Initialize()

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
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())

# Number of simulation steps between renders
render_steps = math.ceil(render_step_size / step_size)

# Simulation loop variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# Maneuver timing parameters
maneuver_start_time = 2.0
maneuver_duration = 4.0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Apply double lane change maneuver based on simulation time
    if time >= maneuver_start_time:
        t_maneuver = time - maneuver_start_time
        if t_maneuver < 1.0:
            # Initial straight driving
            driver_inputs.m_steering = 0.0
        elif t_maneuver < 2.5:
            # First lane change - steer left (positive steering)
            driver_inputs.m_steering = 0.5 * math.sin((t_maneuver - 1.0) * math.pi / 1.5)
        elif t_maneuver < 4.0:
            # Second lane change - steer right (negative steering)
            driver_inputs.m_steering = -0.5 * math.sin((t_maneuver - 2.5) * math.pi / 1.5)
        else:
            driver_inputs.m_steering = 0.0

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)