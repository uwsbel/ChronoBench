import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation (modified as per instructions)
initLoc = chrono.ChVectorD(-5, 0, 0.5)  # Changed from (0,0,1.1) to (-5,0,0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Terrain dimensions
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.1)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size

# Render frame rate
render_step_size = 1.0 / 50  # 50 FPS

# Create the M113 vehicle
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

# Initialize vehicle position and visualization
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Set terrain texture and color
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add a long box obstacle to test vehicle mobility
sys = vehicle.GetSystem()
box = chrono.ChBody()
box.SetPos(chrono.ChVectorD(0, 0, 0.5))  # Positioned in front of the vehicle
box.SetFixed(True)

# Collision model (half-lengths)
box.GetCollisionModel().ClearModel()
box.GetCollisionModel().AddBox(2.0, 1.0, 0.5)
box.GetCollisionModel().BuildModel()
box.SetCollide(True)

# Visualization shape (full lengths)
box_shape = chrono.ChVisualShapeBox(4.0, 2.0, 1.0)
box_shape.SetColor(chrono.ChColor(0.6, 0.3, 0.0))  # Brown color
box.AddVisualShape(box_shape)
sys.Add(box)

# Create visualization system
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)  # Corrected from vehicle.GetVehicle()

# Create driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set input response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Solver settings
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetMass())

# Simulation loop variables
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
vehicle.EnableRealtime(True)

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get and modify driver inputs
    driver_inputs = driver.GetInputs()
    driver_inputs.throttle = 0.8  # Hard-coded throttle value (modified as per instructions)
    
    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1