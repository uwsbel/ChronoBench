import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (modified)
initLoc = chrono.ChVectorD(-5, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization and collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Terrain parameters
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Simulation settings
contact_method = chrono.ChContactMethod_SMC
step_size = 5e-4
render_step_size = 1.0 / 50

# Create the M113 vehicle
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Set chassis collision type (previously missing)
vehicle.SetChassisCollisionType(chassis_collision_type)

# Create terrain
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add a long box obstacle (new)
box = chrono.ChBodyEasyBox(2, 0.5, 0.5, 2000, True, True)  # Dimensions: x=2, y=0.5, z=0.5
box.SetPos(chrono.ChVectorD(0, 0, terrainHeight + 0.25))  # Position on terrain
box.SetBodyFixed(True)
vehicle.GetSystem().Add(box)

# Initialize vehicle with terrain (corrected parameter)
vehicle.Initialize(terrain)

# Setup visualization
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 0.1), 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)

# Driver setup
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# System settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAI_BORWEIN)  # Corrected solver type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Simulation loop setup
print("VEHICLE MASS: ", vehicle.GetMass())  # Fixed incorrect method call
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    # Hard-coded throttle to 0.8 (new)
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.8  # Overriding user input
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1