import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(-15, 0, 0.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Deformable terrain parameters
terrainHeight = 0
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Soil parameters for SCM terrain
soil_shear_modulus = 1e7  # Pa
soil_friction_angle = 30  # degrees
soil_cohesion = 0.0       # Pa
soil_density = 1600       # kg/m^3
soil_damping_ratio = 0.1

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the MAN vehicle, set parameters, and initialize
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Create the deformable SCM terrain
from pychrono.vehicle import DeformableTerrainSCM

terrain = DeformableTerrainSCM(vehicle.GetSystem())
terrain.SetSoilParameters(
    soil_shear_modulus,
    soil_friction_angle * chrono.CH_C_DEG_TO_RAD,
    soil_cohesion,
    soil_density,
    soil_damping_ratio
)

# Initialize terrain with heightmap
heightmap_file = veh.GetDataFile("terrain/heightmaps/sand_heightmap.png")
terrain.AddHeightMap(heightmap_file, terrainLength, terrainWidth, terrainHeight, 1.0)

# Set texture and color for terrain
terrain_texture_file = veh.GetDataFile("terrain/textures/dirt.jpg")
terrain.SetTexture(terrain_texture_file, terrainLength, terrainWidth)
terrain.SetColor(chrono.ChColor(0.4, 0.4, 0.3))  # Dirt color

terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.1), 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system (for steering; throttle is hard-coded)
driver = veh.ChInteractiveDriverIRR(vis)

# Set steering controls (throttle is fixed at 0.8)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Solver and integrator settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Force throttle to 0.8 and retain steering input
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.8  # Hard-coded throttle
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1