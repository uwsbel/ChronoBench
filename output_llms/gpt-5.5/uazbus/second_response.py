import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -----------------------------------------------------------------------------
# Chrono and vehicle data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# -----------------------------------------------------------------------------
# Initial vehicle location and orientation
# Modified: start farther back to allow the double lane change maneuver
# -----------------------------------------------------------------------------
initLoc = chrono.ChVector3d(-40, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts: PRIMITIVES, MESH, or NONE
vis_type = veh.VisualizationType_MESH

# Collision type for chassis: PRIMITIVES, MESH, or NONE
chassis_collision_type = veh.CollisionType_NONE

# Tire model type: RIGID or TMEASY
tire_model = veh.TireModelType_TMEASY

# -----------------------------------------------------------------------------
# Terrain parameters
# -----------------------------------------------------------------------------
terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0

# Chassis point tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50.0  # FPS = 50

# -----------------------------------------------------------------------------
# Create and initialize the UAZBUS vehicle
# -----------------------------------------------------------------------------
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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -----------------------------------------------------------------------------
# Create the rigid terrain
# -----------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, terrainHeight),
        chrono.QUNIT
    ),
    terrainLength,
    terrainWidth
)

# Modified: terrain texture changed from tile4.jpg to concrete.jpg
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# -----------------------------------------------------------------------------
# Create the Irrlicht visualization system
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.AttachVehicle(vehicle.GetVehicle())

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()

# -----------------------------------------------------------------------------
# Automatic driver logic for a double lane change maneuver
# -----------------------------------------------------------------------------
def get_double_lane_change_inputs(time):
    """
    Generate steering, throttle, and braking inputs for a scripted double lane
    change maneuver.

    Steering values are normalized in [-1, 1].
    Throttle and braking values are normalized in [0, 1].
    """

    driver_inputs = veh.DriverInputs()

    steering = 0.0
    throttle = 0.0
    braking = 0.0

    # Accelerate straight before starting the maneuver
    if time < 4.0:
        steering = 0.0
        throttle = 0.65
        braking = 0.0

    # First lane change: steer left
    elif time < 5.0:
        steering = 0.35
        throttle = 0.60
        braking = 0.0

    # Counter-steer to stabilize in the adjacent lane
    elif time < 6.0:
        steering = -0.35
        throttle = 0.55
        braking = 0.0

    # Second lane change: return toward original lane
    elif time < 7.0:
        steering = -0.35
        throttle = 0.55
        braking = 0.0

    # Counter-steer to straighten the vehicle
    elif time < 8.0:
        steering = 0.35
        throttle = 0.50
        braking = 0.0

    # Drive straight after completing the maneuver
    elif time < 11.0:
        steering = 0.0
        throttle = 0.45
        braking = 0.0

    # Begin braking
    elif time < 13.0:
        steering = 0.0
        throttle = 0.0
        braking = 0.35

    # Stronger braking to bring the vehicle to a stop
    else:
        steering = 0.0
        throttle = 0.0
        braking = 0.80

    # Clamp values to valid ranges
    steering = max(-1.0, min(1.0, steering))
    throttle = max(0.0, min(1.0, throttle))
    braking = max(0.0, min(1.0, braking))

    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = throttle
    driver_inputs.m_braking = braking

    return driver_inputs

# -----------------------------------------------------------------------------
# Output vehicle mass
# -----------------------------------------------------------------------------
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get automatic double lane change driver inputs
    driver_inputs = get_double_lane_change_inputs(time)

    # Synchronize modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame counter
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)