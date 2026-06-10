import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 0.0)                             # initial vehicle location (per prompt)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation
step_size = 1e-3                                                       # integration step (s)
sim_end = 10.0                                                         # simulation end time (s)

vis_type = veh.VisualizationType_MESH                                # mesh visuals for track/chassis

vehicle = veh.M113()                                                  # M113 tracked vehicle
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                 # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                       # chassis must be free to move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)               # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                    # tracked driveline (BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                    # shafts engine
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                           # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # spawn pose
vehicle.Initialize()                                                  # build the tracked vehicle

vehicle.SetChassisVisualizationType(vis_type)                        # chassis mesh
vehicle.SetSprocketVisualizationType(vis_type)                       # sprocket mesh
vehicle.SetIdlerVisualizationType(vis_type)                          # idler mesh
vehicle.SetRoadWheelVisualizationType(vis_type)                      # road-wheel mesh
vehicle.SetTrackShoeVisualizationType(vis_type)                      # track-shoe mesh

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED before SCM
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)            # stable tracked-contact solver

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.SCMTerrain(vehicle.GetSystem())                        # deformable Bekker-Wong soft soil
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa.s/m)
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),                                        # chassis-referenced moving patch
    chrono.ChVector3d(0, 0, 0),                                      # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                      # OOBB dimensions (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)          # sinkage heatmap overlay
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),              # SCM height map
    100, 100,                                                        # patch length, width (m)
    -0.9, -0.45,                                                      # min / max height (m): top just under track bottoms at spawn
    0.04,                                                            # grid resolution (m)
)
terrain.SetMeshWireframe(False)                                      # solid mesh
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 80, 80)   # dirt texture (per prompt)

driver = veh.ChDriver(vehicle.GetVehicle())                         # base driver, throttle scripted below
driver.Initialize()                                                  # initialize driver

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                     # tracked-vehicle Irrlicht window
vis.SetWindowTitle("M113 on SCM Deformable Terrain")                # window title
vis.SetWindowSize(1280, 1024)                                       # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera (track point, dist, height)
vis.Initialize()                                                     # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # directional light (vehicle demos)
vis.AttachVehicle(vehicle.GetVehicle())                             # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                        # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
render_every = render_steps                                         # untagged cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                      # physics step counter
while vis.Run():
    time = vehicle.GetSystem().GetChTime()                          # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetThrottle(0.8)                                         # hard-coded throttle (per prompt)
    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                        # advance driver state
    terrain.Synchronize(time)                                       # advance terrain state
    vehicle.Synchronize(time, driver_inputs)                       # tracked: 2-arg synchronize
    vis.Synchronize(time, driver_inputs)                           # update HUD / camera


    driver.Advance(step_size)                                       # step driver
    terrain.Advance(step_size)                                      # step terrain (SCM deformation)
    vehicle.Advance(step_size)                                      # step the wrapper-owned system
    vis.Advance(step_size)                                          # step visualization

    step_number += 1                                               # advance step counter
    realtime_timer.Spin(step_size)                                 # pace to wall clock

    if time >= sim_end:                                            # stop at end time
        break
