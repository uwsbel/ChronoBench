import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 200.0  # Increased from 100.0
terrainWidth = 100.0

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
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

# Create circular path
path = chrono.ChPath()
R = 30.0  # Radius of the circular path
n_points = 50  # Number of points to discretize the circle
for i in range(n_points):
    angle = 2 * math.pi * i / n_points
    x = R * math.cos(angle)
    y = R * math.sin(angle)
    path.AddPoint(chrono.ChVector3d(x, y, 0.6))  # z=0.6 to be above terrain
path.SetClosed(True)

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create path follower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, 5.0)  # Target speed 5 m/s
driver.Initialize()

# Set PID gains for steering
steeringPID = driver.GetSteeringPID()
steeringPID.SetKp(0.8)
steeringPID.SetKi(0.0)
steeringPID.SetKd(0.2)

# Add path visualization spheres
sphere1 = chrono.ChBody()
sphere1.SetBodyFixed(True)
sphere1.SetCollide(False)
vis_shape1 = chrono.ChVisualShapeSphere(0.5)
vis_shape1.SetColor(chrono.ChColor(1, 0, 0))
sphere1.AddVisualShape(vis_shape1)
sphere1.SetPos(path.GetPoint(0))
vehicle.GetSystem().Add(sphere1)

sphere2 = chrono.ChBody()
sphere2.SetBodyFixed(True)
sphere2.SetCollide(False)
vis_shape2 = chrono.ChVisualShapeSphere(0.5)
vis_shape2.SetColor(chrono.ChColor(0, 1, 0))
sphere2.AddVisualShape(vis_shape2)
sphere2.SetPos(path.GetPoint(n_points // 2))
vehicle.GetSystem().Add(sphere2)

# Add sentinel and target visualization spheres
sentinel_sphere = chrono.ChBody()
sentinel_sphere.SetBodyFixed(True)
sentinel_sphere.SetCollide(False)
sentinel_vis = chrono.ChVisualShapeSphere(0.3)
sentinel_vis.SetColor(chrono.ChColor(0, 0, 1))
sentinel_sphere.AddVisualShape(sentinel_vis)
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBody()
target_sphere.SetBodyFixed(True)
target_sphere.SetCollide(False)
target_vis = chrono.ChVisualShapeSphere(0.3)
target_vis.SetColor(chrono.ChColor(1, 0, 1))
target_sphere.AddVisualShape(target_vis)
vehicle.GetSystem().Add(target_sphere)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop variables
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Synchronize driver and get inputs
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  # Constant throttle

    # Update sentinel and target visualization
    sentinel_sphere.SetPos(driver.GetSentinelLocation())
    target_sphere.SetPos(driver.GetTargetLocation())

    # Synchronize other modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)