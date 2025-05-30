import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc1 = chrono.ChVector3d(0, 0, 0.5)
initLoc2 = chrono.ChVector3d(5, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization and collision settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Simulation settings
contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # 50 FPS

# Create the system
system = chrono.ChSystem()
system.SetContactMethod(contact_method)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create and initialize first vehicle
vehicle1 = veh.BMW_E90(system)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()
vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

# Create and initialize second vehicle
vehicle2 = veh.BMW_E90(system)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()
vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicles Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle1.GetVehicle())
vis.AttachVehicle(vehicle2.GetVehicle())

# Create driver systems
driver1 = veh.ChExternalDriver()
driver2 = veh.ChExternalDriver()

# Simulation loop setup
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()
    
    # Sinusoidal steering input for both vehicles
    steering = 0.4 * math.sin(time * 2 * chrono.CH_PI * 0.5)  # 0.5 Hz frequency
    throttle = 0.3
    
    driver1.SetSteering(steering)
    driver1.SetThrottle(throttle)
    driver1.SetBraking(0.0)
    
    driver2.SetSteering(steering)
    driver2.SetThrottle(throttle)
    driver2.SetBraking(0.0)

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Synchronize systems
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    vehicle1.Synchronize(time, driver1.GetInputs(), terrain)
    vehicle2.Synchronize(time, driver2.GetInputs(), terrain)
    vis.Synchronize(time, driver1.GetInputs())

    # Advance systems
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)