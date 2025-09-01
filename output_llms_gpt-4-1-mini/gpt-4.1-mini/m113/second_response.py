import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (changed as per instructions)
initLoc = chrono.ChVector3d(-15, 0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE  # not used for deformable soil

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -------------------------------------------------------------------
# Replace rigid terrain with SCM deformable terrain
# Set SCM soil parameters and initialize with height map and texture

drivingSystem = vehicle.GetVehicle()

scm_terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
scm_terrain.SetPlane(chrono.ChCoordsysd(
    chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  # defined as XY plane at Z=0

# Set soil parameters (example values for a soft soil, adjust as needed)
scm_terrain.SetSoilParameters(
    2e6,  # Bekker Kphi (pressure)
    0,    # Bekker Kc
    1.1,  # Bekker n exponent
    0,    # Mohr cohesive limit (Pa)
    35,   # Mohr friction limit (degrees)
    0.01, # Janosi shear coefficient (m)
    3e7,  # Elastic stiffness (Pa/m)
    3000  # Damping (Pa s/m)
)

# Initialize terrain using a height map file and vertical scale 1.0
height_map_filename = veh.GetDataFile("terrain/height_maps/height_map_64x64_bumps.tif")
scm_terrain.Initialize(height_map_filename, 64, 64, 1.0)

# Set terrain texture ('dirt.jpg' included in Chrono data)
scm_terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 16, 16)

# -------------------------------------------------------------------

# Create the vehicle Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo with SCM Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Solver and integrator settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Hard-code throttle input to 0.8 as per instructions
    # Keep steering and braking from driver keyboard input (optional)
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.8

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    scm_terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    scm_terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1