import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath('/path/to/chrono/data')  # Ensure this path is correct
veh.SetDataPath('/path/to/vehicle/data')  # Ensure this path is correct

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-5, 0, 0.5)  # Changed as per instruction
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 5e-4
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the M113 vehicle, set parameters, and initialize
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

# Create the terrain
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), 
                         terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Add a long box to the system
sys = vehicle.GetSystem()
box_body = chrono.ChBody()
box_body.SetMass(100)
box_body.SetPos(chrono.ChVector3d(0, 0, 1))
box_body.SetBodyFixed(False)

# Create a box shape
box_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(5, 0.5, 0.5))
box_body.AddVisualShape(box_shape)

# Add collision shape
box_mat = chrono.ChContactMaterialSMC()
box_body.GetCollisionModel().ClearModel()
box_body.GetCollisionModel().AddBox(box_mat, 5, 0.5, 0.5, chrono.ChVector3d(0, 0, 0))
box_body.GetCollisionModel().BuildModel()
box_body.SetCollide(True)

sys.Add(box_body)

# Create the vehicle Irrlicht interface
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Simulation loop
step_number = 0
render_frame = 0
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if (step_number % math.ceil(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = veh.Inputs()
    driver_inputs.m_throttle = 0.8  # Hard-coded throttle value as per instruction
    driver_inputs.m_steering = 0
    driver_inputs.m_braking = 0

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1