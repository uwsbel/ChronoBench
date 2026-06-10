import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # vehicle data path

init_loc = chrono.ChVector3d(-15, 0, 0.0)                              # M113 spawn location
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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # BULLET collision BEFORE SCM
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())               # truth diagnostic banner

terrain = veh.SCMTerrain(vehicle.GetSystem())                          # SCM deformable terrain
terrain.SetSoilParameters(
    5301e3,   # Bekker_Kphi   — frictional modulus (Pa), firm soil for the heavy tracked vehicle
    102e3,    # Bekker_Kc     — cohesive modulus
    0.793,    # Bekker_n      — exponent
    1.3e3,    # Mohr_cohesion — cohesive limit (Pa)
    31.1,     # Mohr_friction — friction angle (deg)
    0.0125,   # Janosi_shear  — shear coefficient (m)
    4e8,      # elastic_K     — elastic stiffness (Pa/m)
    3e4,      # damping_R     — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(vehicle.GetChassisBody(),                       # moving patch on chassis (stable OOBB)
                       chrono.ChVector3d(0, 0, 0),
                       chrono.ChVector3d(5, 3, 1))
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),  # SCM from a height map
                   100, 100, -0.5, 0.0, 0.04)                          # length, width, hMin, hMax, resolution
terrain.SetMeshWireframe(False)                                        # solid mesh
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

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
