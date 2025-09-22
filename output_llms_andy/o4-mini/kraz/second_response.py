import pychrono.core as chrono
import pychrono.vehicle as veh
import math

# ----------------------------------------------------------------
#  Chrono data paths
# ----------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------
#  Simulation parameters
# ----------------------------------------------------------------
# Contact method
contact_method = chrono.ChContactMethod_NSC

# Vehicle visualization / collision / tire types
vis_type             = veh.VisualizationType_MESH
chassis_collision    = veh.CollisionType_NONE
tire_model           = veh.TireModelType_TMEASY

# Terrain size
terrainLength = 100.0
terrainWidth  = 100.0

# Time step sizes
step_size        = 1e-3
render_fps       = 50
render_step_size = 1.0 / render_fps
render_steps     = math.ceil(render_step_size / step_size)

# ----------------------------------------------------------------
#  Create and initialize the vehicle
# ----------------------------------------------------------------
# Initial position & orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity

vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetChassisFixed(False)
vehicle.Initialize()

# Visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use the Bullet collision engine
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", vehicle.GetMass())

# ----------------------------------------------------------------
#  Create a rigid flat terrain
# ----------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0,0,0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ----------------------------------------------------------------
#  Create the Irrlicht visual system
# ----------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)

# New track point & camera height
trackPoint = chrono.ChVector3d(3, 0, 2.1)
vis.SetChaseCamera(trackPoint, 25.0, 10.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# ----------------------------------------------------------------
#  Create an interactive driver (but we will override inputs later)
# ----------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
# make keyboard response as smooth as possible (unused for our automated test)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# ----------------------------------------------------------------
#  Simulation loop
# ----------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # -- begin rendering every render_steps
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # -- synchronize modules
    driver.Synchronize(t)
    terrain.Synchronize(t)

    # We override the driver inputs here to perform a simple
    # double-lane-change maneuver:
    #
    #   *  0.0 <= t < 2.0   : straight (no steering)
    #   *  2.0 <= t < 2.8   : steer left  (+0.4)
    #   *  2.8 <= t < 4.4   : steer right (-0.4)
    #   *  4.4 <= t < 5.2   : steer left  (+0.4)
    #   *  5.2 <= t < 5.6   : center      ( 0.0)
    #   *  t >= 5.6         : keep straight
    #
    drv_inputs = driver.GetInputs()  # start from whatever the IRR driver reports
    # override throttle/brake
    drv_inputs.m_throttle = 0.6
    drv_inputs.m_braking  = 0.0
    # override steering for the lane change
    if   t < 2.0:
        drv_inputs.m_steering = 0.0
    elif t < 2.8:
        drv_inputs.m_steering = +0.4
    elif t < 4.4:
        drv_inputs.m_steering = -0.4
    elif t < 5.2:
        drv_inputs.m_steering = +0.4
    elif t < 5.6:
        drv_inputs.m_steering = 0.0
    else:
        drv_inputs.m_steering = 0.0

    # pass the overridden inputs to the vehicle & the visualizer
    vehicle.Synchronize(t, drv_inputs, terrain)
    vis.Synchronize(t, drv_inputs)

    # -- advance all modules by one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # -- real-time step
    realtime_timer.Spin(step_size)
    step_number += 1