import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Initial vehicle location changed to (-50, 0, 0.5)
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain parameters - increased length to 200.0
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 100.0

# Track point for camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle
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

# Create terrain
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

# Create the Irrlicht visualization interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# 1. Replace interactive driver with path-following cruise control driver
# Create a path for the double lane change maneuver using PyChrono API
# For simplicity, approximate the ISO double lane change as a series of waypoints
# The actual ISO double lane change has specific points; here, we define a simplified path

# Define waypoints for the double lane change maneuver
waypoints = [
    chrono.ChVector3d(-50, 0, 0.5),
    chrono.ChVector3d(-40, 3.5, 0.5),
    chrono.ChVector3d(-30, 0, 0.5),
    chrono.ChVector3d(-20, -3.5, 0.5),
    chrono.ChVector3d(-10, 0, 0.5),
    chrono.ChVector3d(0, 0, 0.5)
]

# Create a path object
path = chrono.ChPath()
for wp in waypoints:
    path.AddPoint(wp)

# Create a path-following driver
# Set target speed to 10.0
target_speed = 10.0

# Initialize the PathFollower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, target_speed)

# Configure the steering controller with lookahead distance and gains
look_ahead_distance = 5.0
steering_gain = 0.2  # example gain, tune as needed
speed_gain = 0.1     # example gain, tune as needed

driver.SetLookAheadDistance(look_ahead_distance)
driver.SetSteeringGain(steering_gain)
driver.SetSpeedGain(speed_gain)

# No interactive driver
# driver = veh.ChInteractiveDriverIRR(vis)  # Remove this line

# Set driver to the path follower
# driver.Initialize()  # Already initialized above

# 3. Adjust simulation parameters
# (Already done: initial position and terrain length)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between rendering
render_steps = math.ceil(render_step_size / step_size)

# Initialize timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified intervals
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs from the path follower
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step counter
    step_number += 1

    # Spin in place for real time
    realtime_timer.Spin(step_size)