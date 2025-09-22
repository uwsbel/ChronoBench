import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (changed position)
initLoc = chrono.ChVectorD(-40, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50
contact_vis = False

# Create and initialize the UAZBUS vehicle
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Correct collision system setup
collision_sys = vehicle.GetSystem().GetCollisionSystem()
collision_sys.SetType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain with updated texture
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Double Lane Change')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Simulation control variables
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Compute driver inputs for lane change maneuver
    driver_inputs = veh.ChDriverInputs()
    if time < 2.0:
        driver_inputs.m_steering = 0.0
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_braking = 0.0
    elif 2.0 <= time < 3.0:
        driver_inputs.m_steering = 0.2  # Left turn
        driver_inputs.m_throttle = 0.5
    elif 3.0 <= time < 4.0:
        driver_inputs.m_steering = -0.2  # Right turn
        driver_inputs.m_throttle = 0.5
    elif 4.0 <= time < 6.0:
        driver_inputs.m_steering = 0.0  # Straight
        driver_inputs.m_throttle = 0.5
    else:
        driver_inputs.m_steering = 0.0
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_braking = 1.0  # Brake to stop
    
    # Rendering and simulation steps
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Update and advance simulation
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)