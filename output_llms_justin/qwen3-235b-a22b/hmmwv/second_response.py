import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (on the circular path)
initLoc = chrono.ChVector3d(20, 0, 0.5)  # Start at (20, 0) on the circle
initRot = chrono.ChQuaterniond()
initRot.SetFromRotationAroundZ(math.pi / 2)  # Face along positive Y-axis

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions (increased length to 200)
terrainHeight = 0
terrainLength = 200.0  # Changed from 100 to 200
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

# Create vehicle
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

# Create terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create circular path
path = chrono.ChPath()
arc = chrono.ChLineArc(chrono.ChVector2d(0, 0), 20.0, 0, 2 * math.pi, False)  # Radius 20m
path.AddLine(arc)
path.SetNumPoints(100)

# Add two balls to visualize the path
path_ball1 = chrono.ChBody()
path_ball1.SetBodyFixed(True)
path_ball1.SetPos(chrono.ChVector3d(20, 0, 0.5))
path_ball1.AddVisualShape(chrono.ChVisualShapeSphere(0.5))
vehicle.GetSystem().Add(path_ball1)

path_ball2 = chrono.ChBody()
path_ball2.SetBodyFixed(True)
path_ball2.SetPos(chrono.ChVector3d(0, 20, 0.5))
path_ball2.AddVisualShape(chrono.ChVisualShapeSphere(0.5))
vehicle.GetSystem().Add(path_ball2)

# Create the vehicle visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create path follower driver
driver = veh.ChPathFollowerDriver(vehicle, path, "path", 5.0)  # Target speed 5 m/s
steeringPID = driver.GetSteeringPID()
steeringPID.SetKp(0.8)   # Proportional gain
steeringPID.SetKi(0.0)   # Integral gain
steeringPID.SetKd(0.02)  # Derivative gain
driver.Initialize()

# Create sentinel and target visualization spheres
sentinel_sphere = chrono.ChVisualShapeSphere(0.3)
sentinel_sphere.SetColor(chrono.ChColor(1, 0, 0))  # Red
sentinel_body = chrono.ChBody()
sentinel_body.SetBodyFixed(True)
sentinel_body.AddVisualShape(sentinel_sphere)
vehicle.GetSystem().Add(sentinel_body)

target_sphere = chrono.ChVisualShapeSphere(0.3)
target_sphere.SetColor(chrono.ChColor(0, 0, 1))  # Blue
target_body = chrono.ChBody()
target_body.SetBodyFixed(True)
target_body.AddVisualShape(target_sphere)
vehicle.GetSystem().Add(target_body)

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

    # Get driver inputs and override throttle
    driver_inputs = driver.GetInputs()
    driver_inputs.throttle = 0.3  # Constant throttle

    # Update modules
    driver.Synchronize(time)
    # Update sentinel and target visualization
    sentinel_pos = driver.GetSentinelLocation()
    target_pos = driver.GetTargetLocation()
    sentinel_body.SetPos(sentinel_pos)
    target_body.SetPos(target_pos)
    
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