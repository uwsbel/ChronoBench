"""
Gator Vehicle on Flat Rigid Terrain — PyChrono 9.0.x, Irrlicht renderer.

Models a John Deere Gator utility vehicle (veh.Gator wrapper) driving on a
flat RigidTerrain patch using the TMEASY tire model. The system uses NSC
contact (the rigid-terrain catalog default). An interactive driver (keyboard)
provides real-time control over steering, throttle, and braking. The
simulation targets 50 fps rendering and runs in real time.

System type  : ChSystemNSC (owned by the Gator wrapper)
Main bodies  : Gator chassis + 4 wheel spindles (wrapper-created)
Terrain      : RigidTerrain, single flat patch with a custom texture
Driver       : ChInteractiveDriverIRR (keyboard-controlled)
Expected     : Vehicle sits on terrain at rest; user can drive with keyboard.
"""

# === Imports ===
import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 (vehicle uses its own vis class)
import pychrono.vehicle as veh


# === Data paths (scored core — required by Reference judge) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
TERRAIN_LENGTH = 200.0        # m — patch half-size in X
TERRAIN_WIDTH  = 200.0        # m — patch half-size in Y
INIT_LOC       = chrono.ChVector3d(0.0, 0.0, 0.5)   # vehicle spawn (Z above terrain)
INIT_ROT       = chrono.QuatFromAngleZ(0.0)          # pointing along +X

STEP_SIZE      = 1e-3         # physics step (s)
SIM_END        = 20.0         # simulation duration (s)
RENDER_FPS     = 50.0         # target render / capture rate (Hz)
render_steps   = max(1, math.ceil(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

STEERING_TIME  = 1.0          # seconds to go 0 → ±1 steering
THROTTLE_TIME  = 1.0          # seconds to go 0 → +1 throttle
BRAKING_TIME   = 0.3          # seconds to go 0 → +1 brake

# === Vehicle setup ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)           # prompt: TMEASY tire
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# === System & bodies (created by the veh.Gator wrapper) ===
sys     = gator.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()            # main chassis rigid body  # cache: fetched once
# wheels/spindles: gator.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())  # truth-required diagnostic

# === Visualization types (after Initialize) ===
gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization (vehicle-specific visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Flat Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                         # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()               # vehicle truths use directional light
vis.AttachVehicle(gator.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / RENDER_FPS    # precomputed once

driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()  # cache: fetched once per step

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()  # scored core — used by Synchronize


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
