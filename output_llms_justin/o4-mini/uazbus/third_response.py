import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# --------------------------------------------------------------------
# 1) Set up data paths
# --------------------------------------------------------------------
# Set the global Chrono data path (change this if you installed Chrono data elsewhere)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
# Set the vehicle module path
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# --------------------------------------------------------------------
# 2) Simulation parameters
# --------------------------------------------------------------------
# Initial vehicle location/orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH
# Chassis collision type
chassis_collision_type = veh.CollisionType_NONE
# Tire model: changed from TMEASY to RIGID
tire_model = veh.TireModelType_RIGID

# Terrain geometry
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth  = 100.0

# Camera track point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Time stepping
step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step_size = 1.0 / render_fps

# --------------------------------------------------------------------
# 3) Create and initialize the vehicle
# --------------------------------------------------------------------
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use the BULLET collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# --------------------------------------------------------------------
# 4) Create the terrain
# --------------------------------------------------------------------
# NOTE: for NSC contacts use ChMaterialSurfaceNSC()
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
# Fixed the call to GetChronoDataFile() so it actually finds the Chrono terrain texture:
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# --------------------------------------------------------------------
# 5) Add a fixed box obstacle at (5,0,0.1), size (0.5 x 5 x 0.2)
# --------------------------------------------------------------------
# Use the EasyBox helper to get both collision & visualization in one call
obstacle = chrono.ChBodyEasyBox(
    0.5,  # full X size
    5.0,  # full Y size
    0.2,  # full Z size
    1000, # density (irrelevant since fixed)
    True, # enable collision
    True  # enable visualization
)
obstacle.SetBodyFixed(True)
obstacle.SetPos(chrono.ChVector3d(5, 0, 0.1))
vehicle.GetSystem().Add(obstacle)

# --------------------------------------------------------------------
# 6) Create the driver and Irrlicht visualization
# --------------------------------------------------------------------
# Irrlicht visualization server
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
# attach the *underlying* ChWheeledVehicle object
vis.AttachVehicle(vehicle.GetVehicle())

# Interactive driver (we'll override throttle each frame)
driver = veh.ChInteractiveDriverIRR(vis)
# steering, throttle, braking response times
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# --------------------------------------------------------------------
# 7) Simulation loop
# --------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
step_number = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # periodically render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # 1) read/update driver state
    driver.Synchronize(t)
    inputs = driver.GetInputs()
    # override throttle to a constant 0.5
    inputs.m_throttle = 0.5
    # keep steering and braking at zero if you wish
    # inputs.m_steering = 0.0
    # inputs.m_braking  = 0.0

    # 2) synchronize all modules
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    # 3) advance all modules by one time step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # spin until real time catches up
    realtime_timer.Spin(step_size)
    step_number += 1