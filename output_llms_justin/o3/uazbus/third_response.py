import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
#  Chrono initialisation and global parameters
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle position/orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualisation & collision options
vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE        # fixed enumeration name
tire_model             = veh.TireModelType_RIGID              # << requested change

# Terrain size
terrainHeight = 0
terrainLength = 100.0          # X size
terrainWidth  = 100.0          # Y size

# Chase-cam track point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact / solver options
contact_method = chrono.ChContactMethod_NSC
contact_vis    = False

# Simulation step sizes
step_size      = 1e-3
tire_step_size = step_size

# Render every X seconds
render_step_size = 1.0 / 50.0  # 50 FPS


# ---------------------------------------------------------------------
#  Create the vehicle
# ---------------------------------------------------------------------
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use BULLET collision detection
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
#  Terrain
# ---------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ---------------------------------------------------------------------
#  Added obstacle (box 0.5 × 5 × 0.2 m, fixed in space)
# ---------------------------------------------------------------------
box_length = 0.5
box_width  = 5.0
box_height = 0.2
obstacle   = chrono.ChBodyEasyBox(
    box_length, box_width, box_height,      # size
    1000,                                   # density (irrelevant because fixed)
    True,                                   # visualize
    True,                                   # collide
    patch_mat                               # same contact material
)
obstacle.SetPos(chrono.ChVector3d(5.0, 0.0, box_height / 2.0))   # centre at z = 0.1
obstacle.SetBodyFixed(True)                                      # do not move
vehicle.GetSystem().Add(obstacle)

# ---------------------------------------------------------------------
#  Irrlicht visualisation
# ---------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS – rigid-tire demo with obstacle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ---------------------------------------------------------------------
#  Driver (keyboard + forced constant throttle)
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # sec to go 0 → 1 steering
throttle_time = 1.0   # sec to go 0 → 1 throttle
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# ---------------------------------------------------------------------
#  Simulation loop
# ---------------------------------------------------------------------
render_steps    = math.ceil(render_step_size / step_size)
step_number     = 0
render_frame    = 0
realtime_timer  = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Collect driver inputs and overwrite throttle
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.5            # << constant forward throttle
    # You can still steer/brake from the keyboard if desired.

    # Module synchronisation
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance state
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)