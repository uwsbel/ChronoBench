import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ========== MODIFICATION 1: Increase terrain length ==========
terrainLength = 200.0  # Increased from 100.0 to 200.0
terrainWidth = 100.0
terrainHeight = 0

# ========== MODIFICATION 2: Adjust initial position for circular path ==========
radius = 20.0  # Circle radius
initLoc = chrono.ChVector3d(radius, 0, 0.5)  # Start at circle edge
initRot = chrono.Q_from_AngZ(math.pi/2)  # Orient vehicle tangent to circle

# Visualization and collision settings
vis_type = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Simulation parameters
contact_method = chrono.ChContactMethod_NSC
contact_vis = False
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  # FPS = 50

# Create and initialize vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ========== MODIFICATION 3: Create circular path ==========
path_points = []
for i in range(10):
    angle = 2 * math.pi * i / 9
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0.5))
path = chrono.ChBezierCurve(path_points, True)  # Closed loop

# ========== MODIFICATION 4: Create path-following driver ==========
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "my_path", 0, 10.0)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.5, 0, 0)  # PID gains for steering
driver.GetSpeedController().SetGains(0.4, 0, 0)     # PID gains for speed
driver.Initialize()

# ========== MODIFICATION 5: Create visualization spheres ==========
# Target point sphere (red)
sphere_target = chrono.ChBodyEasySphere(0.5, 1000, True, True)
sphere_target.SetPos(driver.GetSteeringController().GetTargetPoint())
sphere_target.SetBodyFixed(True)
sphere_target.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(sphere_target)

# Sentinel point sphere (green)
sphere_sentinel = chrono.ChBodyEasySphere(0.5, 1000, True, True)
sphere_sentinel.SetPos(driver.GetSteeringController().GetSentinelPoint())
sphere_sentinel.SetBodyFixed(True)
sphere_sentinel.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(sphere_sentinel)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop setup
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # ========== MODIFICATION 6: Driver inputs with constant throttle ==========
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  # Constant throttle override
    driver_inputs.m_braking = 0.0   # Disable braking

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # ========== MODIFICATION 7: Update visualization spheres ==========
    sphere_target.SetPos(driver.GetSteeringController().GetTargetPoint())
    sphere_sentinel.SetPos(driver.GetSteeringController().GetSentinelPoint())

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment counter
    step_number += 1
    realtime_timer.Spin(step_size)