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
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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

# Path and Controller Implementation
# Implement a circular path with a reasonable radius.
path_radius = 20.0
num_points = 100
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0))

# Visualize the path using two balls.
path_balls = []
for i in range(num_points):
    ball = chrono.ChSphereShape()
    ball.GetSphereGeometry().rad = 0.2
    ball.SetPos(path_points[i])
    vis.AddShape(ball)
    path_balls.append(ball)

# Use a constant throttle value of 0.3 and use a PID controller with appropriate gains for steering control
throttle_value = 0.3
pid_gains = chrono.ChVector3d(0.1, 0.01, 0.001)  # P, I, D gains

# Simulation Loop Updates
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (time % render_step_size < step_size) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get current vehicle position and velocity
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_vel = vehicle.GetVehicle().GetPos_dt()

    # Find the closest point on the path to the vehicle
    closest_point = None
    closest_distance = float('inf')
    for i in range(num_points):
        distance = (vehicle_pos - path_points[i]).Length()
        if distance < closest_distance:
            closest_distance = distance
            closest_point = path_points[i]

    # Calculate the steering error
    steering_error = (closest_point - vehicle_pos).Length()

    # Calculate the PID control output
    pid_output = pid_gains.x * steering_error + pid_gains.y * steering_error / step_size + pid_gains.z * steering_error / step_size**2

    # Set the throttle and steering inputs
    vehicle.GetVehicle().SetThrottle(throttle_value)
    vehicle.GetVehicle().SetSteering(pid_output)

    # Advance simulation for one timestep for all modules
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    time += step_size

    # Spin in place for real time to catch up
    chrono.ChRealtimeStepTimer().Spin(step_size)