import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 0.9)                             # initial vehicle location (Z lifts tracks onto SCM surface)
init_rot = chrono.QUNIT                                               # no initial rotation
step_size = 5e-4                                                      # small step keeps the track-shoe contact stable

vehicle = veh.M113()                                                 # M113 tracked vehicle
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                 # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                       # chassis must be free to move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)               # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                    # tracked driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                    # shafts engine model
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                           # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn pose
vehicle.Initialize()                                                 # build the tracked vehicle

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)     # chassis visuals
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)   # track-shoe visuals
vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)    # sprocket visuals
vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)       # idler visuals
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)   # road-wheel visuals

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system (required)
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked contact
vehicle.GetSystem().GetSolver().AsIterative().SetMaxIterations(150)               # more iterations for the stiff track joints

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.SCMTerrain(vehicle.GetSystem())                        # SCM deformable terrain
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa*s/m)
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),            # follow the chassis (stable OOBB projection)
    chrono.ChVector3d(0, 0, 0),          # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),          # OOBB dimensions (m)
)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),   # initialize from a height map
    60, 60, 0, 0.4, 0.04,                                # length, width, hMin, hMax, resolution
)
terrain.SetMeshWireframe(False)                                      # solid deformable mesh
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),   # dirt texture
    80, 80,                                                          # UV tiling
)

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                     # tracked-vehicle Irrlicht system
vis.SetWindowTitle("M113 on SCM terrain")                           # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 6.0, 0.5)            # chase camera on the chassis
vis.Initialize()                                                    # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                            # bind chassis/track visuals

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to vis
render_step_size = 1.0 / 50.0                                       # 50 fps steering time base
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking rate
driver.Initialize()                                                # finalize the driver

time_step = step_size                                              # loop step (s)
sim_end = 10.0                                                     # simulation duration (s)
render_fps = 50.0                                                  # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant

while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = vehicle.GetSystem().GetChTime()
        driver.SetThrottle(0.8)                                   # hard-coded throttle = 0.8
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)                 # tracked: 2-arg synchronize
        vis.Synchronize(time, driver_inputs)
        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)                               # advances the wrapper-owned system
        vis.Advance(time_step)
        if vehicle.GetSystem().GetChTime() >= sim_end:
            break
