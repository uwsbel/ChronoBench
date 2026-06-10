"""
Gator vehicle simulation — PyChrono 9.0.0, Irrlicht, ChSystemNSC.

Models a Gator utility vehicle driving on rigid flat terrain.
Visualization uses PRIMITIVES (not mesh) for chassis, suspension, steering,
wheels, and tires. The chassis has a primitive collision shape (box) added
inline instead of relying on mesh-based chassis collision.
The interactive driver is configured with reduced responsiveness — longer
time-to-full-input for steering, throttle, and braking.

Expected behavior: Gator rests on flat rigid terrain, can be driven
interactively via keyboard with slow control response.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # noqa: F401 (used via veh visual system)
import pychrono.vehicle as veh

# === Constants ===
STEP_SIZE = 5e-4          # physics time step (s)
SIM_END = 20.0            # simulation end time (s)
RENDER_FPS = 50.0         # render frames per second (Hz)
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0    # terrain patch length (m)
TERRAIN_WIDTH = 200.0     # terrain patch width (m)

INIT_LOC = chrono.ChVector3d(0, 0, 0.5)   # initial chassis origin
INIT_ROT = chrono.QuatFromAngleZ(0.0)     # heading: straight ahead

# Driver time-response settings — SLOW (less responsive per prompt)
STEERING_TIME = 4.0   # seconds to go 0 -> +1 steering (default ~1.0 → made slower)
THROTTLE_TIME = 4.0   # seconds to go 0 -> +1 throttle (default ~1.0 → made slower)
BRAKING_TIME  = 2.0   # seconds to go 0 -> +1 braking  (default ~0.3 → made slower)

render_step_size = 1.0 / RENDER_FPS   # precomputed once

# === Data paths (truth-mandatory for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle — veh.Gator() wrapper ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
gator.SetChassisCollisionType(veh.CollisionType_NONE) # primitive collision added below
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# Truth-required vehicle mass diagnostic
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === System & bodies (created by the veh.Gator wrapper) ===
sys = gator.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody() # main chassis rigid body  # cache: fetched once, reused

# Collision system (REQUIRED for contact/terrain)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization types — PRIMITIVES (not mesh, per prompt) ===
gator.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === Chassis collision — primitive box (per prompt: simple, not mesh) ===
cmat = chrono.ChContactMaterialNSC()
cmat.SetFriction(0.8)
cmat.SetRestitution(0.0)
chassis.AddCollisionShape(
    chrono.ChCollisionShapeBox(cmat, 2.5, 1.2, 0.4),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.3), chrono.QUNIT),
)
chassis.EnableCollision(True)
sys.GetCollisionSystem().BindAll()  # rebuild after post-init shape addition

# === Terrain — RigidTerrain, flat, NSC ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
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

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — Primitives Visualization, Slow Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 7.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()    # vehicle truth uses directional light
vis.AttachVehicle(gator.GetVehicle())

# === Driver — ChInteractiveDriverIRR (scored-core default) ===
driver = veh.ChInteractiveDriverIRR(vis)
# Slow deltas → less responsive controls (per prompt)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
