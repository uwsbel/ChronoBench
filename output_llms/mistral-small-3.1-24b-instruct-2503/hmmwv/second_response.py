import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # increased size in X direction
terrainWidth = 100.0   # size in Y direction

# Point tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced()  could be another choice here
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the circular path
radius = 20.0
center = chrono.ChVector3d(0, 0, 0)
path = chrono.ChPath()
for angle in range(0, 360, 10):
    x = center.x + radius * math.cos(math.radians(angle))
    y = center.y + radius * math.sin(math.radians(angle))
    path.AddPoint(chrono.ChVector3d(x, y, 0))

# Create the path follower driver
driver = veh.ChPathFollowerDriverFULL(vehicle.GetVehicle(), path, tire_step_size)
driver.SetSteeringControllerGain(1.0)
driver.SetSteeringControllerTimeConstant(0.3)
driver.SetThrottleControllerGain(0.3)
driver.SetThrottleControllerTimeConstant(0.5)
driver.Initialize()

# Visualization of controller points
sentinel = chrono.ChSphereShape()
sentinel.GetSphereGeometry().radius = 0.5
sentinel.GetMaterialSurface().SetFriction(0.6)
sentinel.GetMaterialSurface().SetRestitution(0.1)
sentinel_body = chrono.ChBodyEasySphere(0.5, 1000, True, True)
sentinel_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
sentinel_body.SetBodyFixed(True)
sentinel_body.AddVisualShape(sentinel)
vehicle.GetSystem().Add(sentinel_body)

target = chrono.ChSphereShape()
target.GetSphereGeometry().radius = 0.5
target.GetMaterialSurface().SetFriction(0.6)
target.GetMaterialSurface().SetRestitution(0.1)
target_body = chrono.ChBodyEasySphere(0.5, 1000, True, True)
target_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
target_body.SetBodyFixed(True)
target_body.AddVisualShape(target)
vehicle.GetSystem().Add(target_body)

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sentinel and target points
    sentinel_body.SetPos(driver.GetSentinelPoint())
    target_body.SetPos(driver.GetTargetPoint())

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)