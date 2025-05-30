import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# --------------------------------------------------------------------------
# 1) DATA PATHS
# --------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# --------------------------------------------------------------------------
# 2) INITIAL VEHICLE LOCATION AND VISUAL/COLLISION TYPES
# --------------------------------------------------------------------------
# Changed initial location as requested:
initLoc = chrono.ChVector3d(-5, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# --------------------------------------------------------------------------
# 3) TERRAIN PARAMETERS
# --------------------------------------------------------------------------
terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0

# --------------------------------------------------------------------------
# 4) CAMERA TRACK POINT
# --------------------------------------------------------------------------
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.1)

# --------------------------------------------------------------------------
# 5) CONTACT, STEP SIZES, RENDER RATE
# --------------------------------------------------------------------------
contact_method   = chrono.ChContactMethod_SMC
step_size        = 5e-4
tire_step_size   = step_size
render_fps       = 50
render_step_size = 1.0 / render_fps

# --------------------------------------------------------------------------
# 6) CREATE AND INITIALIZE THE M113 VEHICLE
# --------------------------------------------------------------------------
vehicle = veh.M113()
vehicle.SetContactMethod(contact_method)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

# set chassis collision mode (was defined but never used)
vehicle.SetChassisCollisionType(chassis_collision_type)

# initialize at the new position
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

# set all visualizations to mesh
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetIdlerWheelVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# use the Bullet collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# optional: make the system use a more robust solver
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# print out the vehicle mass
print("VEHICLE MASS = ", vehicle.GetVehicle().GetMass())

# --------------------------------------------------------------------------
# 7) CREATE THE RIGID TERRAIN
# --------------------------------------------------------------------------
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight),
                                            chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# --------------------------------------------------------------------------
# 8) ADD A LONG BOX INTO THE SYSTEM AS AN OBSTACLE
# --------------------------------------------------------------------------
# here we create a box 5m long x 1m wide x 0.5m tall, density 1000 kg/m3,
# with collision and visualization enabled
box = chrono.ChBodyEasyBox(5.0, 1.0, 0.5, 1000, True, True)
# place it a bit ahead of the vehicle center
box.SetPos(chrono.ChVector3d(-2.0, 0.0, 0.25))
box.SetBodyFixed(False)
vehicle.GetSystem().Add(box)

# --------------------------------------------------------------------------
# 9) CREATE THE IRRLICHT VISUALIZATION
# --------------------------------------------------------------------------
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113 Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 9.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --------------------------------------------------------------------------
# 10) CREATE THE DRIVER
# --------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)
# limit how fast keyboard inputs can change:
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# --------------------------------------------------------------------------
# 11) RUN THE SIMULATION
# --------------------------------------------------------------------------
# how many steps between frames
render_steps = math.ceil(render_step_size / step_size)

step_number  = 0
render_frame = 0

# enable real time to keep near realtime execution
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # 1) collect driver inputs
    drv_inputs = driver.GetInputs()
    # override throttle to a constant 0.8
    drv_inputs.m_throttle = 0.8

    # 2) synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, drv_inputs)
    vis.Synchronize(time, drv_inputs)

    # 3) advance simulation by one step for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1