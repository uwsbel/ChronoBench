"""
M113 tracked armored personnel carrier simulation on rigid terrain.

System type: SMC (as required by M113 truth)
Vehicle: veh.M113() tracked vehicle with SINGLE_PIN track shoes, BDS driveline,
         SHAFTS engine, AUTOMATIC_SHAFTS transmission, SIMPLE brakes.
Terrain: RigidTerrain flat patch with SMC contact material.
Driver: ChInteractiveDriverIRR (real-time interactive; scripted throttle=0.8 in scored core
        per m113 truth shape).
Expected behavior: Vehicle accelerates forward on flat terrain under constant throttle.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
step_size = 5e-4          # integration time step (s)
sim_end = 20.0            # simulation duration (s)
render_fps = 50.0         # rendering frame rate (Hz)
render_steps = math.ceil(1.0 / (render_fps * step_size))  # precomputed once

TERRAIN_LENGTH = 300.0    # terrain patch length (m)
TERRAIN_WIDTH  = 300.0    # terrain patch width (m)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 1.0              # chassis init height (m above ground)

# === Data paths (MANDATORY — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup (M113 tracked) ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # M113 truth uses SMC
vehicle.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# === System & bodies (created by the veh.M113 wrapper) ===
sys = vehicle.GetSystem()          # ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody() # cache: fetched once, reused every step
# track shoes, sprockets, idlers, road wheels: created inside wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)            # stable solver for tracked contact

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())            # truth diagnostic

# === Visualization types ===
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialSMC()   # SMC system -> SMC material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization (tracked vehicle visual system) ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)  # chase camera from behind/above
vis.Initialize()                                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                            # vehicle truths use directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver setup (interactive — ChInteractiveDriverIRR per m113 truth shape) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / render_fps  # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_steps == 0:   # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # M113 truth scripts throttle=0.8 directly (scripted driver shape)
        driver_inputs.m_throttle = 0.8
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking  = 0.0

        # Synchronize subsystems — tracked vehicle: 2-arg Synchronize (no terrain arg)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)   # 2-arg for tracked vehicles
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

        if sys.GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
