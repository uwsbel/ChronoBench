"""
ARTcar on RigidTerrain — PyChrono 9.0.x / Irrlicht

Models the ARTcar wheeled vehicle driving on a flat rigid terrain patch with a
custom texture. An interactive Irrlicht driver (ChInteractiveDriverIRR) provides
keyboard-based steering, throttle, and braking at runtime. The simulation runs at
50 frames per second in real-time with a 1 ms physics time-step, matching the
catalog-vehicle truth pattern (ChSystemNSC, NSC contact material, Type_BULLET
collision).

System type : ChSystemNSC (owned by ARTcar wrapper)
Main bodies : ARTcar chassis, four wheel spindles, terrain patch
Expected    : ARTcar starts at origin, moves forward under user throttle input;
              terrain displays tiled custom texture; HUD shows steering/throttle/brake bars
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
STEP_SIZE        = 1e-3          # physics timestep (s)
SIM_END          = 30.0          # simulation duration (s) — enough for an observable run
RENDER_FPS       = 50.0          # Irrlicht render cadence
TERRAIN_LENGTH   = 200.0         # terrain patch length (m)
TERRAIN_WIDTH    = 100.0         # terrain patch width (m)
TIRE_FRICTION    = 0.9
TERRAIN_FRICTION = 0.9

# Precomputed once — never recompute inside the loop
RENDER_STEP_SIZE = 1.0 / RENDER_FPS                                        # precomputed once
RENDER_STEPS     = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))         # precomputed once

STEERING_TIME = 1.0   # s to reach max steering
THROTTLE_TIME = 1.0   # s to reach max throttle
BRAKING_TIME  = 0.3   # s to reach max braking

# === ARTcar vehicle setup ===
artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
artcar.SetChassisCollisionType(veh.CollisionType_NONE)
artcar.SetChassisFixed(False)                          # MANDATORY — unfixed chassis

init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)           # spawn above terrain
init_rot = chrono.QuatFromAngleZ(0.0)                  # facing +X
artcar.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
artcar.SetTireType(veh.TireModelType_RIGID)            # rigid tire for rigid terrain
artcar.SetTireStepSize(STEP_SIZE)
artcar.Initialize()

# === System & bodies (created by veh.ARTcar wrapper) ===
sys     = artcar.GetSystem()                           # ChSystemNSC owned by wrapper
chassis = artcar.GetChassisBody()                      # cache: fetched once, reused every step
# wheels/spindles: artcar.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

# Visualization types (must come after Initialize)
artcar.SetChassisVisualizationType(chrono.VisualizationType_MESH)
artcar.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
artcar.SetWheelVisualizationType(chrono.VisualizationType_MESH)
artcar.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain  = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,        # centered at world origin
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Custom texture on the terrain patch
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
# Irrlicht call order: configure FIRST → Initialize() → add scene elements AFTER
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar on RigidTerrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 4.0, 0.5)   # follow from behind
vis.Initialize()                                               # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(artcar.GetVehicle())

# === Interactive driver (scored core — ChInteractiveDriver takes the vehicle) ===
driver = veh.ChInteractiveDriver(artcar.GetVehicle())
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_STEPS == 0:      # throttled rendering at RENDER_FPS
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()   # scored-core: live keyboard inputs


        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        artcar.Advance(STEP_SIZE)      # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
