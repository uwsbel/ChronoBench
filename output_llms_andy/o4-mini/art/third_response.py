import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ----------------------------------------------------------------
# Utility: set the Chrono and vehicle data paths
# ----------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------
# Initial vehicle location and orientation
# ----------------------------------------------------------------
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# ----------------------------------------------------------------
# Visualization and collision types
# ----------------------------------------------------------------
vis_type             = veh.VisualizationType_MESH
chassis_collision   = veh.CollisionType_NONE
tire_model          = veh.TireModelType_TMEASY

# ----------------------------------------------------------------
# Terrain dimensions
# ----------------------------------------------------------------
terrainLength = 100.0
terrainWidth  = 100.0

# ----------------------------------------------------------------
# Simulation settings
# ----------------------------------------------------------------
contact_method    = chrono.ChContactMethod_NSC
step_size         = 1e-3
tire_step_size    = step_size
render_fps        = 50
render_step_size  = 1.0 / render_fps

# ----------------------------------------------------------------
# Create and initialize the ARTcar vehicle
# ----------------------------------------------------------------
vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# --- YOUR REQUESTED PERFORMANCE TUNING ---
vehicle.SetMaxMotorVoltageRatio(0.26)     # was 0.16
vehicle.SetStallTorque(0.4)               # was 0.3
vehicle.SetTireRollingResistance(0.03)    # was 0.06

vehicle.Initialize()

# set up visualization for all subsystems
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# use Bullet for collision
vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystemType_BULLET)

# ----------------------------------------------------------------
# Create a rigid flat terrain
# ----------------------------------------------------------------
mat_NS = chrono.ChMaterialSurfaceNSC()
mat_NS.SetFriction(0.9)
mat_NS.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    mat_NS,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ----------------------------------------------------------------
# Set up the Irrlicht visualization
# ----------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0,0,0.2), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)                # attach the vehicle

# ----------------------------------------------------------------
# Interactive driver (keyboard)
# ----------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
steer_time   = 1.0   # time to go from 0->1 steering
throttle_time= 1.0   # time to go from 0->1 throttle
brake_time   = 0.3   # time to go from 0->1 braking
driver.SetSteeringDelta(render_step_size / steer_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / brake_time)
driver.Initialize()

# ----------------------------------------------------------------
# Simulation loop
# ----------------------------------------------------------------
print("VEHICLE MASS:", vehicle.GetChassisBody().GetMass())

realtime_timer = chrono.ChRealtimeStepTimer()
render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # render once every 'render_steps'
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # driver inputs
    driver_inputs = driver.GetInputs()

    # synchronize all modules
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, driver_inputs, terrain)
    vis.Synchronize(t, driver_inputs)

    # advance all modules by one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # spin in real time
    realtime_timer.Spin(step_size)
    step_number += 1