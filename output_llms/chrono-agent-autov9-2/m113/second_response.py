"""M113 tracked vehicle driving on SCM deformable (Bekker-Wong) terrain.

Model: the M113 armored personnel carrier — a tracked vehicle with single-pin
track shoes, a SHAFTS engine and a BDS (basic) tracked driveline. It is spawned
at world location (-15, 0, 0.64) on a deformable SCM soft-soil patch whose surface
is built from a height map and textured with dirt. The vehicle is driven
open-loop at a constant 0.8 throttle (no steering, no braking), so it accelerates
forward across the soft soil, leaving sinkage ruts under the tracks.

System type: NSC (Non-Smooth Contact). A single-pin track is numerically
unstable under SMC, so non-smooth contact is used for both the tracked vehicle
and the SCM terrain. The Bullet collision system is required because the tracked
running gear, terrain and track shoes all carry collision geometry; SCMTerrain
also queries the collision system via ray-casts to compute sinkage.

Expected behavior: the M113 starts at rest at x=-15 and, under constant 0.8
throttle, builds forward speed and translates in +x while the tracks sink into
and deform the SCM surface.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants (geometry / physics) — no bare literals downstream ===
TIME_STEP = 2e-3                       # integration step (s); SCM + tracks
SIM_END = 8.0                          # simulation duration (s)
RENDER_FPS = 50.0                      # review-video frame rate

INIT_X = -15.0                         # vehicle spawn x (world)
INIT_Y = 0.0                           # vehicle spawn y (world)
INIT_Z = 0.74                          # chassis-origin spawn height above terrain (m)
THROTTLE = 0.8                         # hard-coded constant throttle

# SCM patch + height map
SCM_SIZE_X = 80.0                      # terrain length along x (m)
SCM_SIZE_Y = 30.0                      # terrain width along y (m)
SCM_DELTA = 0.1                        # SCM grid resolution (m)
SCM_HMIN = 0.0                         # height-map min height (m)
SCM_HMAX = 0.1                         # height-map max height (m) — gentle relief

# SCM Bekker-Wong soil parameters (firm, high-shear soil so the tracks grip)
SOIL_KPHI = 5e6                        # Bekker frictional modulus (Pa)
SOIL_KC = 2e4                          # Bekker cohesive modulus
SOIL_N = 1.0                           # Bekker exponent
SOIL_COHESION = 2e4                    # Mohr cohesive limit (Pa)
SOIL_FRICTION = 45.0                   # Mohr friction angle (deg)
SOIL_JANOSI = 0.001                    # Janosi shear coefficient (m) — small = grip at low slip
SOIL_ELASTIC_K = 4e7                   # elastic stiffness (Pa/m)
SOIL_DAMPING_R = 3e4                   # vertical damping (Pa.s/m)

# Derived constants (precomputed once)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QUNIT

# === Vehicle (M113 tracked APC) — wrapper creates system + chassis + tracks ===
# The veh.M113 wrapper builds and owns its ChSystemNSC, the chassis rigid body,
# the sprockets/idlers/road wheels and the two single-pin track assemblies.
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # single-pin track: NSC, not SMC
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)      # basic tracked driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === System & collision (owned by the M113 wrapper) ===
sys = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()        # main chassis rigid body  # cache: fetched once, reused every step
tracked = vehicle.GetVehicle()            # ChTrackedVehicle handle  # cache: fetched once, reused every step
# Bullet collision is REQUIRED: tracks + road wheels + terrain carry collision
# geometry and SCM ray-casts the collision system to compute sinkage.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# track-shoe counts per side (precomputed once for the per-step force buffers)
num_shoes_left = tracked.GetNumTrackShoes(veh.LEFT)    # precomputed once
num_shoes_right = tracked.GetNumTrackShoes(veh.RIGHT)  # precomputed once

# === Terrain (SCM deformable Bekker-Wong soft soil from a height map) ===
# SCMTerrain needs the collision system to already exist (set above).
terrain = veh.SCMTerrain(sys)
terrain.SetSoilParameters(
    SOIL_KPHI,        # Bekker_Kphi
    SOIL_KC,          # Bekker_Kc
    SOIL_N,           # Bekker_n
    SOIL_COHESION,    # Mohr_cohesion
    SOIL_FRICTION,    # Mohr_friction
    SOIL_JANOSI,      # Janosi_shear
    SOIL_ELASTIC_K,   # elastic_K
    SOIL_DAMPING_R,   # damping_R
)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 20, 20)
# Initialize the deformable surface from a height-map image.
terrain.Initialize(
    chrono.GetChronoDataFile("vehicle/terrain/height_maps/bump64.bmp"),
    SCM_SIZE_X, SCM_SIZE_Y,
    SCM_HMIN, SCM_HMAX,
    SCM_DELTA,
)

# === Driver (open-loop constant throttle 0.8) ===
# Plain DriverInputs struct written each step — hard-coded full-on throttle,
# no steering, no braking.
driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = THROTTLE
driver_inputs.m_steering = 0.0
driver_inputs.m_braking = 0.0

# === Visualization === tracked-vehicle Irrlicht window: chase cam + sky + lights + grid
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("M113 on SCM deformable terrain")
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 1.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(INIT_X - 8.0, -8.0, 4.0), INIT_LOC)
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(INIT_X, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(tracked)

# === Main loop === render at cadence; advance the tracked-vehicle subsystem stack


frame = 0
try:
    while vis.Run() and tracked.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = tracked.GetChTime()
            shoe_forces_left = veh.TerrainForces(num_shoes_left)
            shoe_forces_right = veh.TerrainForces(num_shoes_right)
            driver_inputs.m_throttle = THROTTLE   # hard-coded constant throttle
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, shoe_forces_left, shoe_forces_right)
            vis.Synchronize(sim_time, driver_inputs)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)     # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if tracked.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + plot, then drop frame folders
