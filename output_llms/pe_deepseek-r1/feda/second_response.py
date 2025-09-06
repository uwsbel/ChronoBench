import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Corrected data path setup
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Changed initial location: (-50, 0, 0.5) instead of (0, 0, 0.5)
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model
tire_model = veh.TireModelType_TMEASY

# 3. Increased terrain length from 100.0 to 200.0
terrainHeight = 0
terrainLength = 200.0  # Increased size in X direction
terrainWidth = 100.0

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

# Create and initialize the FEDA vehicle
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle - Double Lane Change')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# 1. Replaced interactive driver with path-follower system
# Create double lane change path (ISO standard)
points = [
    chrono.ChVector3d(-50, 0, 0.5),    # Start point
    chrono.ChVector3d(-25, 0, 0.5),     # Approach first cone
    chrono.ChVector3d(0, -3.5, 0.5),    # First lane change (left)
    chrono.ChVector3d(25, 0, 0.5),      # Center again
    chrono.ChVector3d(50, 3.5, 0.5),    # Second lane change (right)
    chrono.ChVector3d(75, 0, 0.5),      # Final center
    chrono.ChVector3d(100, 0, 0.5)      # End point
]
path = chrono.ChBezierCurve(points)

# Create steering controller with look-ahead distance
steering_controller = veh.ChSteeringControllerPID()
steering_controller.SetGains(0.5, 0, 0)  # PID gains (Kp, Ki, Kd)
steering_controller.SetLookAheadDistance(5.0)  # Look-ahead distance

# Create speed controller
speed_controller = veh.ChSpeedControllerPID()
speed_controller.SetGains(0.4, 0, 0)  # PID gains

# Create path-follower driver system
driver = veh.ChPathFollowerDriver(
    vehicle.GetVehicle(),   # Target vehicle
    path,                   # Path to follow
    "double_lane_change",   # Path name
    0,                      # Target speed (will be overridden)
    steering_controller,    # Steering controller
    speed_controller        # Speed controller
)
driver.SetSpeed(10.0)  # Set target speed to 10.0 m/s
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
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

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin for real-time synchronization
    realtime_timer.Spin(step_size)