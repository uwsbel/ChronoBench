import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# SetChronoDataPath
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle initialization parameters
initLoc = chrono.ChVector3d(-40, 0, 0.5)  # Changed initial position
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization settings
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Create a ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the UAZBUS vehicle
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.Initialize()

# Set visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

# Changed texture to concrete.jpg
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Visualization setup
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Lane Change Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Driver setup
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)  # render_step_size / steering_time
driver.SetThrottleDelta(0.02)  # render_step_size / throttle_time
driver.SetBrakingDelta(0.006)  # render_step_size / braking_time
driver.Initialize()

# Simulation parameters
step_size = 1e-3
render_step_size = 1.0 / 50
render_steps = math.ceil(render_step_size / step_size)

# Double lane change maneuver parameters
# Time (s), Steering (-1 to 1), Throttle (0 to 1)
maneuver_times = [
    (0, 0, 0.5),          # Start with straight driving
    (2, 0.5, 0.5),        # Start first lane change
    (4, -0.5, 0.5),       # Countersteer for double lane change
    (6, 0, 0.5),          # Straighten
    (8, 0, 0)             # Braking and stopping
]

current_maneuver = 0
total_time = 0

# Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs based on maneuver schedule
    if time >= maneuver_times[current_maneuver][0]:
        steering = maneuver_times[current_maneuver][1]
        throttle = maneuver_times[current_maneuver][2]
        if current_maneuver < len(maneuver_times) - 1:
            current_maneuver += 1
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = throttle
    driver_inputs.m_braking = 0 if throttle > 0 else 1
    
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
    
    # Render scene
    if (total_time % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    total_time += 1
    realtime_timer.Spin(step_size)