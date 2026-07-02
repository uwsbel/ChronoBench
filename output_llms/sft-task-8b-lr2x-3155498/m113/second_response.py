import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

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

# Create SCM deformable terrain
soil_density = 1950.0
soil_friction = 0.5
soil_restitution = 0.2
soil_rolling_friction = 0.01
soil_cohesion = 5000.0
soil_shear = 15000.0
soil_normal_stiff = 100000.0
soil_tangent_stiff = 80000.0
soil_damping = 0.3

terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilProperties(soil_density, soil_friction, soil_restitution, soil_rolling_friction, soil_cohesion, soil_shear, soil_normal_stiff, soil_tangent_stiff, soil_damping)
terrain.Initialize(100.0, 100.0, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), 100, 100)

# Create height map
height_map = []
for i in range(101):
    row = []
    for j in range(101):
        row.append(0.0)
    height_map.append(row)
terrain.SetHeightMap(height_map, 100, 100, 1.0)

# Set texture
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 100, 100)

# Create the vehicle Irrlicht interface

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 SCM Terrain Demo')
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
# ------------------------------
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Hard-coded throttle value
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = driver.GetInputs().m_steering
    driver_inputs.m_throttle = 0.8
    driver_inputs.m_braking = driver.GetInputs().m_braking

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    # Increment frame number
    step_number += 1