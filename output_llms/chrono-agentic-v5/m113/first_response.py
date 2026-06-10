"""M113 tracked vehicle on flat rigid terrain (real-time Irrlicht demo).

Models the M113 armored personnel carrier as a Chrono tracked vehicle (single-pin
track shoes, shafts engine + automatic shafts transmission, BDS driveline, simple
brakes) driven over a flat rigid-terrain patch with defined friction/restitution.
System type: SMC (the M113 truth uses an SMC contact model; the BARZILAI-BORWEIN
solver stabilizes the many track-shoe contacts). An interactive Irrlicht driver
controls the vehicle; the loop synchronizes and advances driver, terrain, vehicle,
and visualization every timestep in real time.

Expected behavior: the tank rests on the terrain at the chassis reference height,
its tracks engage the ground, and under throttle it drives forward in a straight
line without the chassis sinking through the terrain.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived spawn pose
time_step = 1e-3                       # integration step (s); tracked contact needs small dt
sim_end = 10.0                         # total simulated time (s)
render_fps = 50.0                      # review-render cadence (frames/s)
render_step_size = 1.0 / render_fps    # precomputed once
render_steps = math.ceil(render_step_size / time_step)   # precomputed once: steps per frame

TERRAIN_LENGTH = 100.0                 # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0                  # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.8                 # ground friction coefficient
TERRAIN_RESTITUTION = 0.01             # ground bounciness
TERRAIN_YOUNG = 2e7                    # SMC contact stiffness (Pa)

INIT_HEIGHT = 0.8                      # chassis-origin height above terrain at rest (m)
init_loc = chrono.ChVector3d(0, 0, INIT_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === M113 tracked vehicle (wrapper owns its ChSystem)
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# Visualization detail for every tracked subsystem.
vis_type = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)

# === System & bodies (created by the veh.M113 wrapper) ===
sys = vehicle.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused below
# tracks/sprockets/idlers/road-wheels + their joints are built inside the wrapper.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)            # stable track contact
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === flat rigid patch with defined friction / restitution
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()      # SMC system -> SMC material
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch_mat.SetYoungModulus(TERRAIN_YOUNG)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === tracked-vehicle Irrlicht window: sky + camera + light
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 8.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                  # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive Irrlicht driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time synchronize/advance of driver, terrain, vehicle, vis

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)   # tracked: 2-arg, no terrain
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)                 # advances the wrapper-owned system
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)             # match wall-clock to sim time
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise

# === Post-processing === assemble the review video + physics plot from captured data
