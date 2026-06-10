import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ----------------------------------------------------------------------
# Set Chrono data paths (adjust if necessary)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ----------------------------------------------------------------------
# Vehicle initial location and orientation
# *** MODIFIED: changed from (0,0,0.5) to (-50,0,0.5) ***
initLoc = chrono.ChVector3d(-50.0, 0.0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Tire model type (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# ----------------------------------------------------------------------
# Rigid terrain parameters
# *** MODIFIED: increased terrain length from 100.0 to 200.0 ***
terrainHeight = 0.0
terrainLength = 200.0   # size in X direction
terrainWidth  = 100.0   # size in Y direction

# Point on the chassis that the chase camera tracks
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method (NSC = non‑smooth contact)
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames (50 FPS)
render_step_size = 1.0 / 50.0

# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# Create the rigid terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# ----------------------------------------------------------------------
# Create the visualisation system (Irrlicht)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# ----------------------------------------------------------------------
# *** NEW: Create a path‑following driver ***

# Load the ISO double‑lane‑change path (assumed to be present in Chrono data)
path = veh.LoadPathFromFile(veh.GetDataFile('paths/dlc.trail'))

# Create the path‑follower driver
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)

# Configure steering controller (look‑ahead distance and gains)
driver.SetLookAheadDistance(5.0)                # look‑ahead distance [m]
driver.SetSteeringControllerGains(2.0, 0.0, 0.0) # Kp, Ki, Kd

# Configure speed controller (gains)
driver.SetSpeedControllerGains(0.5, 0.0, 0.0)    # Kp, Ki, Kd

# Set target speed for the path‑follower
driver.SetTargetSpeed(10.0)                      # desired speed [m/s]

# Initialize the driver
driver.Initialize()

# ----------------------------------------------------------------------
# Output vehicle mass
print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

# ----------------------------------------------------------------------
# Simulation loop
render_steps = math.ceil(render_step_size / step_size)

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

    # Get driver inputs (computed by the path‑follower driver)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Spin to maintain real‑time performance
    realtime_timer.Spin(step_size)