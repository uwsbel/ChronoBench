import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Change 1: Increase terrain length
terrainLength = 200.0

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0 
terrainWidth = 100.0 

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50 

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

# Change 2: Implement a circular path and visualize it using two balls
path_radius = 30
path_center = chrono.ChVector3d(0, 0, 0)
num_path_points = 100
path = veh.Path()
for i in range(num_path_points):
    angle = 2 * math.pi * i / num_path_points
    point = path_center + chrono.ChVector3d(path_radius * math.cos(angle), path_radius * math.sin(angle), 0.02)
    path.AddPoint(point)

# Visualize the path using two balls
ball1 = chrono.ChBodyEasySphere(1, 1000, True, False)
ball1.SetPos(path.GetPoint(0))
ball1.SetBodyFixed(True)
vis.Add(ball1)

ball2 = chrono.ChBodyEasySphere(1, 1000, True, False)
ball2.SetPos(path.GetPoint(num_path_points//2))
ball2.SetBodyFixed(True)
vis.Add(ball2)

# Change 3: Use a constant throttle value and a PID controller for steering control
throttle_value = 0.3
path_follower = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "my_path", 30)
path_follower.GetSteeringController().SetLookAheadDistance(5)
path_follower.GetSteeringController().SetGains(0.5, 0, 0)

# Simulation loop updates
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = path_follower.GetInputs()
    driver_inputs.throttle = throttle_value

    # Update modules (process inputs from other modules)
    path_follower.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Change 5: Visualize the sentinel and target points
    sentinel = path_follower.GetSteeringController().GetSentinelPoint()
    target = path_follower.GetSteeringController().GetTargetPoint()
    sentinel_vis = vis.GetSceneManager().getSceneNodeFromName("sentinel")
    if sentinel_vis is None:
        sentinel_vis = vis.GetSceneManager().addSphereSceneNode(0.1)
        sentinel_vis.setName("sentinel")
    sentinel_vis.setPosition(irr.vector3df(sentinel.x, sentinel.y, sentinel.z))
    target_vis = vis.GetSceneManager().getSceneNodeFromName("target")
    if target_vis is None:
        target_vis = vis.GetSceneManager().addSphereSceneNode(0.1)
        target_vis.setName("target")
    target_vis.setPosition(irr.vector3df(target.x, target.y, target.z))

    # Advance simulation for one timestep for all modules
    path_follower.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# No modifications needed in the triple backticks, the code is already provided above.