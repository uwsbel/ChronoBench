import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # vehicle data path

init_loc = chrono.ChVector3d(-5, 0, 0.5)                               # M113 spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT, no rotation

step_size = 5e-4                                                        # M113 truth step size
render_step_size = 1.0 / 50.0                                          # 50 FPS render cadence

vehicle = veh.M113()                                                   # TRACKED vehicle (no tires)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                         # chassis must move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)                 # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                      # tracked-vehicle driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                      # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                             # simple brakes
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
vehicle.Initialize()                                                   # build the tracked vehicle

vis_type = veh.VisualizationType_MESH                                  # mesh visualization
vehicle.SetChassisVisualizationType(vis_type)                          # chassis mesh
vehicle.SetSprocketVisualizationType(vis_type)                         # sprocket mesh
vehicle.SetIdlerVisualizationType(vis_type)                            # idler mesh
vehicle.SetIdlerWheelVisualizationType(vis_type)                       # idler wheel mesh
vehicle.SetRoadWheelVisualizationType(vis_type)                        # road wheel mesh
vehicle.SetTrackShoeVisualizationType(vis_type)                        # track shoe mesh

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # BULLET collision (contact scene)
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())               # truth diagnostic banner

terrain = veh.RigidTerrain(vehicle.GetSystem())                        # rigid terrain on the vehicle's system
patch_mat = chrono.ChContactMaterialSMC()                             # SMC material (pairs with SMC method)
patch_mat.SetFriction(0.9)                                             # terrain friction
patch_mat.SetRestitution(0.01)                                        # terrain restitution
patch_mat.SetYoungModulus(2e7)                                        # SMC stiffness
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)    # 100x100 flat patch at origin
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                         # patch color
terrain.Initialize()                                                   # build the terrain

# Long box obstacle to test vehicle mobility (driven over by the tracks)
box_mat = chrono.ChContactMaterialSMC()                               # SMC material for the obstacle
box_mat.SetFriction(0.9)                                              # obstacle friction
box_mat.SetRestitution(0.01)                                         # obstacle restitution
box_mat.SetYoungModulus(2e7)                                         # obstacle stiffness
obstacle = chrono.ChBodyEasyBox(0.4, 5.0, 0.2, 1000, True, True, box_mat)  # long, low box (X x Y x Z)
obstacle.SetPos(chrono.ChVector3d(5.0, 0, 0.1))                       # ahead of the vehicle, top above ground
obstacle.SetFixed(True)                                               # fixed obstacle ramp
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.3, 0.1))    # brown obstacle
vehicle.GetSystem().AddBody(obstacle)                                  # add obstacle to the system

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                       # tracked-vehicle Irrlicht vis
vis.SetWindowTitle('M113 Demo')                                        # window title
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.1), 9.0, 1.5)            # trackpoint, chase dist, height
vis.Initialize()                                                       # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo after Initialize
vis.AddSkyBox()                                                        # sky box
vis.AddLightDirectional()                                             # single directional light (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                                # bind the vehicle to the vis

driver = veh.ChInteractiveDriverIRR(vis)                               # driver system for vehicle control
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking rate
driver.Initialize()                                                    # initialize the driver

vehicle.GetVehicle().EnableRealtime(True)                              # real-time execution (M113 truth)

render_steps = math.ceil(render_step_size / step_size)                # physics steps per rendered frame
step_number = 0                                                        # physics step counter
while vis.Run():                                                       # SCORED CORE = plain truth loop
    time = vehicle.GetSystem().GetChTime()                             # current sim time

    if step_number % render_steps == 0:                                # throttled rendering once per frame
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                 # current driver inputs
    driver_inputs.m_throttle = 0.8                                     # hard-coded throttle during the loop

    driver.Synchronize(time)                                           # sync driver
    terrain.Synchronize(time)                                          # sync terrain
    vehicle.Synchronize(time, driver_inputs)                           # 2-arg sync for TRACKED vehicle
    vis.Synchronize(time, driver_inputs)                               # sync vis

    driver.Advance(step_size)                                          # advance driver
    terrain.Advance(step_size)                                         # advance terrain
    vehicle.Advance(step_size)                                         # advance vehicle (steps the system)
    vis.Advance(step_size)                                             # advance vis

    step_number += 1                                                   # next step
