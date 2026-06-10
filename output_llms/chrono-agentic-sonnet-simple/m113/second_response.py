import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 1.0)                            # initial vehicle position
init_rot = chrono.QuatFromAngleZ(0)                                  # no initial yaw

vehicle = veh.M113()                                                 # tracked vehicle wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)                 # M113 truth uses SMC
vehicle.SetChassisFixed(False)                                       # MANDATORY — must not be fixed
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)               # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)                    # BDS driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)                    # shafts engine
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shafts transmission
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)                           # simple brake model
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # set spawn position
vehicle.Initialize()                                                 # build the vehicle

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)     # chassis visual
vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH)          # sprocket meshes
vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH)             # idler meshes
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)         # road wheel meshes
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)         # track shoe meshes

system = vehicle.GetSystem()                                         # get the owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)           # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.SCMTerrain(system)                                     # SCM deformable terrain
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),                                        # attach to chassis, not tracks
    chrono.ChVector3d(0, 0, 0),                                      # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                      # OOBB dims (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)           # sinkage heatmap overlay
# Use terrain3.bmp: gentle terrain with only 0.15 m max elevation variation
terrain.Initialize(veh.GetDataFile("terrain/height_maps/terrain3.bmp"),
                   100, 100, -0.1, 0.1, 0.05)                       # heightmap: 100x100 m, ±0.1 m, 5 cm res
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

vis = veh.ChTrackedVehicleVisualSystemIrrlicht()                     # tracked vehicle Irrlicht vis
vis.SetWindowTitle("M113 on SCM Terrain")                            # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)         # chase camera behind chassis
vis.Initialize()                                                     # FIRST — build Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # Chrono logo
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                            # directional light (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                              # bind chassis/track visual assets

# Scripted driver: hard-code throttle = 0.8 per prompt
class ThrottleDriver(veh.ChDriver):
    def __init__(self, veh_handle):
        super().__init__(veh_handle)

    def Synchronize(self, time):
        self.SetThrottle(0.8)                                        # hard-coded throttle = 0.8
        self.SetSteering(0.0)                                        # drive straight
        self.SetBraking(0.0)

driver = ThrottleDriver(vehicle.GetVehicle())                        # scripted throttle driver
driver.Initialize()                                                  # finalize driver

step_size = 5e-4                                                     # time step (s) — small dt for tracked contact
sim_end = 20.0                                                       # simulation duration (s)
render_fps = 50.0                                                    # render frame rate
render_every = max(1, round(1.0 / (render_fps * step_size)))         # physics steps per render frame

step_number = 0                                                      # step counter for render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time pacer
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                        # current sim time

    if step_number % render_every == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)                         # 2-arg for tracked vehicle
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)                                       # internally advances the system
    vis.Advance(step_size)

    step_number += 1                                                 # advance step counter
    realtime_timer.Spin(step_size)                                   # real-time pacing
