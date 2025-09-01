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

# Rigid terrain replaced with SCM deformable terrain
# SCM terrain parameters
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.6)
patch_mat.SetRestitution(0.3)
patch_mat.SetDeformation(0.2)
patch_mat.SetPressure(1.0)

# Initialize SCM terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    100, 100)

# Set texture and color for the terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

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

# Set visualization for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set time response for steering, throttle, and braking
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)  # Hard-coded to 0.8
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Solver and integrator settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize and advance simulation
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    
    # Apply constant throttle
    driver.Throttle(0.8)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1