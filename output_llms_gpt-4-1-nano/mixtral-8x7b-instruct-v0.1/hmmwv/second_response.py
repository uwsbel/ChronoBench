import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np
from pychrono.vehicle import ChPathSteeringController

# Add this line to import the ChLineBezier class
from pychrono.geometry import ChLineBezier

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50


# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced()  could be another choice here
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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())
# Visualization of controller points (sentinel & target)
ballS = vis.GetSceneManager().addSphereSceneNode(0.1)
ballT = vis.GetSceneManager().addSphereSceneNode(0.1)
ballS.getMaterial(0).EmissiveColor = irr.SColor(0, 255, 0, 0)
ballT.getMaterial(0).EmissiveColor = irr.SColor(0, 0, 255, 0)


# Left circle path
path_radius = 20.0
path_center = chrono.ChVector3d(0, 0, terrainHeight)
path = veh.CirclePath(path_center, path_radius, 0, 2 * math.pi, 100)

path_asset = chrono.ChVisualShapeLine()
path_asset.SetLineGeometry(ChLineBezier(path))
path_asset.SetName("test path")
path_asset.SetNumRenderPoints(max(2 * len(path), 400))
patch.GetGroundBody().AddVisualShape(path_asset)

# Create the PID lateral controller
steeringPID = ChPathSteeringController(path)
steeringPID.SetLookAheadDistance(5)
steeringPID.SetGains(0.8, 0, 0)
steeringPID_output = 0
throttle_value = 0.3

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get driver inputs
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = np.clip(steeringPID_output, -1.0, +1.0)
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_braking = 0.0

    pS = steeringPID.GetSentinelLocation()
    pT = steeringPID.GetTargetLocation()
    ballS.setPosition(irr.vector3df(pS.x, pS.y, pS.z))
    ballT.setPosition(irr.vector3df(pT.x, pT.y, pT.z))

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    steeringPID_output = steeringPID.Advance(vehicle.GetRefFrame(), time, step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)