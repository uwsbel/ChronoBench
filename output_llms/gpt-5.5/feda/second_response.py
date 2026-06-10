import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Initial vehicle location and orientation
# Modified so the ISO double lane change fits on the terrain patch
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts: PRIMITIVES, MESH, or NONE
vis_type = veh.VisualizationType_MESH

# Collision type for chassis: PRIMITIVES, MESH, or NONE
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model: RIGID or TMEASY
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0.0
terrainLength = 200.0   # Increased from 100.0 to fit the double lane change
terrainWidth = 100.0

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50.0  # FPS = 50

# -------------------------------------------------------------------------
# Create the FEDA vehicle, set parameters, and initialize
# -------------------------------------------------------------------------

vehicle = veh.FEDA()
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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -------------------------------------------------------------------------
# Create the terrain
# -------------------------------------------------------------------------

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -------------------------------------------------------------------------
# Create the vehicle Irrlicht visualization system
# -------------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA vehicle - ISO Double Lane Change")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -------------------------------------------------------------------------
# Create path-follower / cruise-control driver
# -------------------------------------------------------------------------

target_speed = 10.0  # m/s

# ISO double lane change path.
# The vehicle starts at x = -50 so the full maneuver fits within the
# terrain patch, which spans x = [-100, 100].
path = veh.DoubleLaneChangePath(
    initLoc,
    13.5,   # approach/entry segment length
    4.0,    # lane offset/width
    11.0,   # transition/control length
    50.0,   # exit segment length
    True    # left-right-left maneuver
)

driver = veh.ChPathFollowerDriver(
    vehicle.GetVehicle(),
    path,
    "ISO_double_lane_change",
    target_speed,
)

# Steering controller configuration
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)

# Speed controller configuration
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)

driver.Initialize()

# -------------------------------------------------------------------------
# Output vehicle mass
# -------------------------------------------------------------------------

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# -------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update driver first so current inputs are available
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Real-time pacing
    realtime_timer.Spin(step_size)