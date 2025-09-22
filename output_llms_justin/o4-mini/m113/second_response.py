import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# --------------------------------------------------------------------
# 1) Initialize Chrono paths
# --------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# --------------------------------------------------------------------
# 2) Simulation parameters
# --------------------------------------------------------------------
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15.0, 0.0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH
# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Point for the chase camera to track (on the chassis)
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Time stepping
step_size = 5e-4
render_step_size = 1.0 / 50  # 50 FPS

# --------------------------------------------------------------------
# 3) Create and initialize the tracked vehicle (M113)
# --------------------------------------------------------------------
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(
    veh.TransmissionModelType_AUTOMATIC_SHAFTS
)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

# Set chassis collision type (this was previously declared but never used)
vehicle.SetChassisCollisionType(chassis_collision_type)

# Initialize the vehicle
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

# Visualization
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Use Bullet for collision detection
vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET
)

# --------------------------------------------------------------------
# 4) Create an SCM deformable terrain
# --------------------------------------------------------------------
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())

# A plausible set of soil parameters:
#   Bek        Cohesion  Kphi      nu    Kc        Damping   vsink  compliance
terrain.SetSoilParameters(
    2e6,       # Bek      [N/m^(n+1)]
    0.0,       # Cohesion [Pa]
    1e7,       # Kphi     [N/m^(n+1)]
    0.3,       # nu       [–]
    2e5,       # Kc       [N/m^(n+1)]
    2e3,       # Damping  [Pa·s/m]
    2e-1,      # vsink    [m/s]
    0.0        # compliance
)

# Height‐map initialization
height_map_file = chrono.GetChronoDataFile("terrain/height_maps/heightmap.png")
# The Initialize call arguments here are:
#   (file, length_X, width_Y, vertical_scale)
terrain.Initialize(height_map_file, 100.0, 100.0, 0.01)

# Texture it with “dirt”
terrain.SetTexture(
    chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), 200, 200
)

# --------------------------------------------------------------------
# 5) Create the Irrlicht visualization system
# --------------------------------------------------------------------
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 SCM Terrain Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)  # for tracked vehicles, pass 'vehicle' directly

# --------------------------------------------------------------------
# 6) Driver (Irrlicht-based, but we will override throttle)
# --------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # 1s to go 0 → ±1
driver.SetThrottleDelta(render_step_size / 1.0)   # 1s to go 0 → 1
driver.SetBrakingDelta(render_step_size / 0.3)    # 0.3s to go 0 → 1
driver.Initialize()

# --------------------------------------------------------------------
# 7) Solver configuration
# --------------------------------------------------------------------
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetMass())

# Compute how many steps between renders
render_steps = math.ceil(render_step_size / step_size)

# --------------------------------------------------------------------
# 8) Simulation loop
# --------------------------------------------------------------------
step_number = 0
vis.EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render once every render_steps
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get the current driver inputs, but override throttle = 0.8
    inputs = driver.GetInputs()  
    inputs.m_throttle = 0.8

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs)
    vis.Synchronize(time, inputs)

    # Advance all modules by one time step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1