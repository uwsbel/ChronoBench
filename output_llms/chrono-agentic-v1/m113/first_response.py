"""
M113 tracked vehicle simulation on rigid terrain using PyChrono 9.0.x with Irrlicht.

System type: ChSystemNSC (NSC contact method, owned by the veh.M113 wrapper)
Vehicle: M113 tracked vehicle (veh.M113)
  - TrackShoeType_SINGLE_PIN
  - DrivelineTypeTV_BDS
  - EngineModelType_SHAFTS + TransmissionModelType_AUTOMATIC_SHAFTS
  - BrakeType_SIMPLE
Terrain: RigidTerrain with NSC material (friction=0.9, restitution=0.01)
Driver: ChInteractiveDriverIRR (real-time interactive, keyboard control)
Visualization: ChTrackedVehicleVisualSystemIrrlicht with chase camera

Expected behavior: M113 sits on flat rigid terrain, responds to keyboard
throttle/steering input, moves forward under engine power in real-time.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Data path (vehicle data bundled with Chrono) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
# Simulation timing
TIME_STEP = 5e-4              # 0.5 ms — tracked vehicle needs small step for stability
SIM_END = 20.0                # seconds
RENDER_FPS = 50.0             # frames per second for review video
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame (precomputed once)

# Terrain parameters
TERRAIN_LENGTH = 100.0        # m
TERRAIN_WIDTH  = 100.0        # m
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Vehicle initial position — above terrain (z=0 flat patch, M113 chassis ~0.5 m above ground)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 1.0                  # chassis origin height above terrain

# Chase camera parameters
CHASE_OFFSET = chrono.ChVector3d(0.0, 0.0, 1.75)  # track point on chassis
CHASE_DIST   = 9.0            # camera distance behind vehicle
CHASE_HEIGHT = 0.5            # camera height offset

# Driver response times (seconds to reach full input)
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3

# === Vehicle setup ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

# Initial position and orientation (level, facing +X)
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (no rotation)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# NO SetTireType for tracked vehicle — it has no tires

# Visualization types (called after Initialize) — VisualizationType lives in veh namespace
vis_type = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)

# === System & bodies (created by veh.M113 wrapper) ===
system = vehicle.GetSystem()              # ChSystemNSC owned by the M113 wrapper
chassis = vehicle.GetChassisBody()       # cache: main chassis rigid body, fetched once
# Track shoes, sprockets, idlers, road wheels created inside the wrapper
# Terrain body: RigidTerrain patch added below
# Joints: suspension + track links created inside the wrapper

# Collision system — REQUIRED for tracked vehicle on terrain contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Solver for tracked vehicle stability
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (ChTrackedVehicleVisualSystemIrrlicht) ===
# Configure window and chase camera before Initialize; add scene elements after
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle - PyChrono 9.0")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_OFFSET, CHASE_DIST, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())  # ChTrackedVehicleVisualSystemIrrlicht expects ChVehicle*

# === Driver (ChInteractiveDriverIRR — takes the visual system) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / STEERING_TIME)
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / BRAKING_TIME)
driver.Initialize()

# === Recording setup (review-only) ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()  # cache: current sim time

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems — tracked vehicle: 2-arg Synchronize (no terrain arg)
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs)   # 2-arg for tracked vehicles
        vis.Synchronize(sim_time, driver_inputs)


        # Advance all subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle.Advance(TIME_STEP)   # advances the wrapper-owned ChSystem (DO NOT also call DoStepDynamics)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)  # real-time pacing

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
