import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
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

# Terrain parameters
terrainHeight = 0
terrainLength = 200.0   # Increased from 100.0 to 200.0
terrainWidth = 100.0

# Camera tracking point
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

# --- PATH AND CONTROLLER IMPLEMENTATION ---

# Define a circular path
circle_center = chrono.ChVector3d(0, 0, 0.5)
circle_radius = 40.0  # Reasonable radius to fit in terrain
circle_normal = chrono.ChVector3d(0, 0, 1)
path = chrono.ChPath()
# Create a circle using ChLineArc
arc = chrono.ChLineArc(circle_center, circle_radius, 0, 2 * math.pi, circle_normal)
path.Initialize()
path.AddSubLine(arc)
path.Set_closed(True)

# Visualize the path using two balls (start and halfway point)
start_point = arc.GetPoint(0)
halfway_point = arc.GetPoint(0.5)

# Add spheres to the Irrlicht scene for visualization
sphere_shape_start = chrono.ChSphereShape()
sphere_shape_start.GetSphereGeometry().rad = 1.0
sphere_start = chrono.ChBodyEasySphere(1.0, 1000, True, True)
sphere_start.SetPos(start_point)
sphere_start.SetBodyFixed(True)
sphere_start.AddVisualShape(sphere_shape_start)
vehicle.GetSystem().Add(sphere_start)

sphere_shape_half = chrono.ChSphereShape()
sphere_shape_half.GetSphereGeometry().rad = 1.0
sphere_half = chrono.ChBodyEasySphere(1.0, 1000, True, True)
sphere_half.SetPos(halfway_point)
sphere_half.SetBodyFixed(True)
sphere_half.AddVisualShape(sphere_shape_half)
vehicle.GetSystem().Add(sphere_half)

# Create the path-follower driver
sentinel_dist = 6.0
target_dist = 20.0
controller = veh.ChPathFollowerDriver(
    vehicle, path, "my_path", 0.3,  # constant throttle
    sentinel_dist, target_dist
)
# PID gains for steering (tune as needed)
controller.GetSteeringController().SetGains(0.8, 0.0, 0.3)
controller.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# --- Visualization of Controller Points (sentinel and target) ---
# We'll use two spheres and update their positions each step

sentinel_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, True)
sentinel_sphere.SetBodyFixed(True)
sentinel_sphere.AddVisualShape(chrono.ChSphereShape())
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, True)
target_sphere.SetBodyFixed(True)
target_sphere.AddVisualShape(chrono.ChSphereShape())
vehicle.GetSystem().Add(target_sphere)

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs from path-follower controller
    driver_inputs = controller.GetInputs()

    # Update modules (process inputs from other modules)
    controller.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Update visualization of sentinel and target points
    sentinel = controller.GetSentinelLocation()
    target = controller.GetTargetLocation()
    sentinel_sphere.SetPos(sentinel)
    target_sphere.SetPos(target)

    # Advance simulation for one timestep for all modules
    controller.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)