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
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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

# Create a path using two balls
path_radius = 20.0
num_balls = 100
balls = []
for i in range(num_balls):
    angle = 2 * math.pi * i / num_balls
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    ball = chrono.ChSphereShape()
    ball.GetSphereGeometry().SetRadius(0.5)
    ball.SetPos(chrono.ChVector3d(x, y, 1.0))
    balls.append(ball)

# Create a PID controller for steering control
Kp = 0.1
Ki = 0.01
Kd = 0.001
controller = veh.ChPathFollowerPID(vehicle.GetVehicle(), Kp, Ki, Kd)
controller.SetTargetSpeed(5.0)

# Set the throttle value
throttle_value = 0.3

# Simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (int(time / render_step_size) % 2 == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update modules (process inputs from other modules)
    controller.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, chrono.ChDriverInputs(throttle_value, controller.GetSteering()))

    # Advance simulation for one timestep for all modules
    controller.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Visualize the path using two balls
    for ball in balls:
        vis.DrawShape(ball, chrono.ChColor(1.0, 0.0, 0.0))

    # Visualize the sentinel and target points
    sentinel_point = chrono.ChVector3d(controller.GetSentinelPoint().x(), controller.GetSentinelPoint().y(), 1.0)
    target_point = chrono.ChVector3d(controller.GetTargetPoint().x(), controller.GetTargetPoint().y(), 1.0)
    vis.DrawSphere(sentinel_point, 0.5, chrono.ChColor(0.0, 1.0, 0.0))
    vis.DrawSphere(target_point, 0.5, chrono.ChColor(0.0, 0.0, 1.0))