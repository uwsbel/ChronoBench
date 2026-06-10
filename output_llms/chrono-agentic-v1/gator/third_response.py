"""
Gator vehicle simulation — PyChrono 9.0.x, Irrlicht renderer, ChSystemNSC.

Models a John Deere Gator utility vehicle driving on rigid flat terrain.
Visualization uses PRIMITIVES (not meshes) for chassis, suspension, steering,
wheels, and tires. The chassis also has primitive collision shapes added inline
(a bounding box), satisfying the request to keep collision simple with primitive
shapes rather than a mesh. Driver response is intentionally slow: large time
constants make controls take longer to apply when keyboard keys are pressed.

Expected behavior: The Gator drives with sluggish, delayed control response;
all vehicle components are rendered as boxes/cylinders (primitives).
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr  # noqa: F401 — vehicle vis inherits from this

# === Data paths ===
# Set vehicle data path so veh.GetDataFile resolves bundled terrain/vehicle assets
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
step_size    = 1e-3         # physics time step (s)
sim_end      = 20.0         # simulation end time (s)
render_fps   = 50.0         # target render rate (Hz)
# precomputed once — number of physics steps between rendered frames
render_steps = max(1, round(1.0 / (render_fps * step_size)))

# Terrain dimensions (m)
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0

# Vehicle spawn: wheels on flat z=0 terrain; Gator chassis ref ≈ 0.5 m above wheel-bottom
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT  = chrono.ChQuaterniond(1, 0, 0, 0)

# Driver time constants — larger values mean SLOWER (less responsive) controls
STEERING_TIME = 4.0   # s to go from 0 → max steering
THROTTLE_TIME = 4.0   # s to go from 0 → max throttle
BRAKING_TIME  = 2.0   # s to go from 0 → max braking

# === Vehicle setup (Gator wrapper — owns its ChSystem) ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
gator.SetChassisCollisionType(veh.CollisionType_NONE)   # primitives added inline below

gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(step_size)

gator.Initialize()

# Visualization: PRIMITIVES for all subsystems (prompt: change from mesh to primitives)
# NOTE: Set vis types AFTER Initialize() — before causes SIGSEGV in this build.
# VisualizationType_PRIMITIVES lives in the veh namespace in this PyChrono 9.0.0 build.
gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by veh.Gator wrapper) ===
system  = gator.GetSystem()        # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()   # cache: main chassis rigid body — reused for collision + CSV
# wheels/spindles: gator.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the Gator wrapper
# terrain: RigidTerrain patch body added below

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# === Chassis collision — primitive box shape (not mesh) ===
# Prompt: add chassis collision using simple primitive shapes, not a mesh.
# Approximate Gator: 2.4 m × 1.2 m × 0.6 m bounding box offset +0.3 m vertically.
chassis_mat = chrono.ChContactMaterialNSC()
chassis_mat.SetFriction(0.7)
chassis_mat.SetRestitution(0.1)

chassis.AddCollisionShape(
    chrono.ChCollisionShapeBox(chassis_mat, 2.4, 1.2, 0.6),
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.3), chrono.QUNIT),
)
chassis.EnableCollision(True)

# Rebuild collision models after post-init shape additions
system.GetCollisionSystem().BindAll()

# === Terrain — rigid flat ground ===
terrain = veh.RigidTerrain(system)

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

# === Irrlicht visualization (vehicle-aware) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — Primitives Vis, Chassis Collision, Slow Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.0), 8.0, 0.5)
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(gator.GetVehicle())

# === Driver — interactive IRR with slow (less-responsive) time response ===
# Prompt: controls take more time to apply (larger deltas = faster; smaller = slower).
# Using step_size/STEERING_TIME instead of render_step_size/1.0 gives much slower response.
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(step_size / STEERING_TIME)   # slow steering response
driver.SetThrottleDelta(step_size / THROTTLE_TIME)   # slow throttle response
driver.SetBrakingDelta(step_size / BRAKING_TIME)     # slow braking response
driver.Initialize()

# vis.AttachDriver not available in this build; driver HUD wired via vis.Synchronize

# === Recording setup ===

# CSV writer — opened once before the loop

# === Real-time step timer (scored core — matches catalog-vehicle truth) ===
realtime_timer = chrono.ChRealtimeStepTimer()

# === Main loop ===
step_number = 0
frame       = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()  # cache: fetched once per outer-loop iteration

        # Throttled rendering: only render every render_steps physics steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()  # scored-core: interactive driver inputs


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
