import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set Chrono data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-50, 0, 0.5)  # Modified initial location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0
terrainLength = 200.0  # Modified terrain length
terrainWidth = 100.0

# Track point for camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the FEDA vehicle, set parameters, and initialize
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
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create path for double lane change maneuver
path = chrono.ChPath()
path.SetName("Double Lane Change Path")

# Define path points for ISO double lane change (example points)
path_points = [
    chrono.ChVector3d(-50, 0, 0.5),
    chrono.ChVector3d(-40, 0, 0.5),
    chrono.ChVector3d(-30, 2, 0.5),
    chrono.ChVector3d(-20, 0, 0.5),
    chrono.ChVector3d(-10, -2, 0.5),
    chrono.ChVector3d(0, 0, 0.5),
    chrono.ChVector3d(10, 2, 0.5),
    chrono.ChVector3d(20, 0, 0.5),
    chrono.ChVector3d(30, -2, 0.5),
    chrono.ChVector3d(40, 0, 0.5),
    chrono.ChVector3d(50, 0, 0.5)
]

for point in path_points:
    path.AddPoint(point)

# Create path follower driver
driver = veh.ChPathFollowerDriver(path)
driver.SetTargetSpeed(10.0)  # Target speed of 10.0 m/s
driver.SetLookAheadDistance(5.0)  # Look-ahead distance of 5 meters

# Configure steering controller gains
driver.SetSteeringGain(1.0)  # Proportional gain for steering
driver.SetSteeringIntegralGain(0.1)  # Integral gain for steering

# Configure speed controller gains
driver.SetSpeedGain(1.0)  # Proportional gain for speed
driver.SetSpeedIntegralGain(0.1)  # Integral gain for speed

# Initialize driver
driver.Initialize()
driver.AttachVehicle(vehicle.GetVehicle())

# Initialize simulation frame counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update driver
    driver.Synchronize(time)
    driver.Advance(step_size)

    # Update vehicle
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vehicle.Advance(step_size)

    # Update terrain and visualization
    terrain.Synchronize(time)
    terrain.Advance(step_size)
    vis.Synchronize(time, driver.GetInputs())
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)