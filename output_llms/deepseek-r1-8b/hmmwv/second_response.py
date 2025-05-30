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
terrainLength = 200.0  # size in X direction (increased from 100.0)
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

# Path parameters
circular_radius = 50.0  # Radius of the circular path (meters)
circular_center = chrono.ChVector3d(0, 0, 0.5)  # Initial position of the path
path_points = []  # To store path points

def generateCircularPath():
    angle = 0
    while True:
        x = circular_radius * math.cos(angle)
        y = circular_radius * math.sin(angle)
        z = 0.5  # Keep the path at the same height as the vehicle
        point = chrono.ChVector3d(x, y, z)
        path_points.append(point)
        angle += 0.1  # Increment angle for next point
        if angle > 2 * math.pi:
            break

generateCircularPath()

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set up the path follower
path_follower = veh.ChPathFollowerVehicleSystem()
path_follower.SetVehicle(vehicle)
path_follower.SetPathPoints(path_points)
path_follower.SetPathIndex(0)  # Start at the first point
path_follower.SetMaxSteeringAngle(1.0)
path_follower.SetMaxSpeed(5.0)  # Adjust speed as needed
path_follower.Initialize()

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
vis.SetWindowTitle('HMMWV Path Follower Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Add visualization for path
start_point = path_points[0]
end_point = path_points[-1]
start_sphere = vis.AddSphere(start_point, 0.5, chrono.ChColor(0, 1, 0))
end_sphere = vis.AddSphere(end_point, 0.5, chrono.ChColor(1, 0, 0))

# Path follower controller
path_follower_controller = veh.ChPathFollowerPIDController()
path_follower_controller.SetProportionalGain(0.1)
path_follower_controller.SetIntegralGain(0.1)
path_follower_controller.SetDerivativeGain(0.05)
path_follower_controller.Initialize()

# Simulation loop
driver = None  # Remove interactive driver since we're using path follower

# Set time response for steering and throttle inputs (not used in this implementation)
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules
    vehicle.Synchronize(time)
    terrain.Synchronize(time)
    path_follower.Synchronize(time)
    vis.Synchronize(time)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)  # Remove this if driver is not used
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    path_follower.Advance(step_size)

    # Calculate path follower state
    current_pos = vehicle.GetVehicle().GetPosition()
    current_angle = path_follower.GetCurrentAngle()

    # Calculate target direction (commented out for brevity)
    # target_dir = path_follower.GetTargetDirection()
    # target_dist = path_follower.GetTargetDistance()

    # Update PID controller and get steering angle
    steering_angle = path_follower_controller.GetSteeringAngle(current_pos, current_angle)
    steering_delta = steering_angle * (render_step_size / 2)  # Normalize to [-1, 1]

    # Update vehicle steering
    vehicle.GetVehicle().SetSteering(steering_delta)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)