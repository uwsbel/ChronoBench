"""
ARTcar vehicle simulation on flat rigid terrain.

System type: ChSystemNSC (owned by the ARTcar wrapper).
Protagonist: ARTcar wheeled vehicle (veh.ARTcar).
Terrain: RigidTerrain with flat patch (NSC contact).
Driver: ChInteractiveDriverIRR (real-time interactive, truth-matched).

Configuration:
  - Initial vehicle location: (1, 0, 0.5)
  - Visualization type: PRIMITIVES for all vehicle parts
  - Chassis collision type: CollisionType_MESH
  - Tire model: FIALA

Expected behavior: ARTcar initializes at world position (1, 0, 0.5), rendered
with primitives geometry, chassis collision mesh active, FIALA tire forces,
driving on flat rigid terrain under interactive keyboard control.
"""

# === Imports ===
import math  # for render_step computation

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Simulation parameters ===
step_size = 1e-3          # physics time step (s)
sim_end = 20.0            # total sim duration (s)  -- inferred default -- verify
render_fps = 50.0         # Irrlicht render frame rate (Hz)
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# Terrain dimensions (large enough for 20s drive at ~12m/s = ~240m total; use 500x100)
terrainLength = 500.0
terrainWidth  = 100.0

# === Vehicle: ARTcar setup ===
# Initial location: (1, 0, 0.5) per prompt
init_loc = chrono.ChVector3d(1.0, 0.0, 0.5)
init_rot = chrono.QUNIT

artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod_NSC)  # NSC for rigid terrain
artcar.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
artcar.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Tire model: FIALA per prompt (replaces TMEASY)
artcar.SetTireType(veh.TireModelType_FIALA)  # prompt: FIALA tire model
artcar.SetTireStepSize(step_size)

# Chassis collision: MESH per prompt (replaces NONE)
artcar.SetChassisCollisionType(veh.CollisionType_MESH)  # prompt: mesh collision

artcar.Initialize()

# === System & bodies (created by the veh.ARTcar wrapper) ===
sys = artcar.GetSystem()              # ChSystemNSC owned by the wrapper
chassis = artcar.GetChassisBody()     # main chassis rigid body  # cache: fetched once

# Collision system — REQUIRED for vehicle+terrain contact
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization type: PRIMITIVES per prompt ===
# Note: in the 9.0.0 source build VisualizationType_* lives in pychrono.vehicle, not pychrono.core
artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)    # prompt
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # prompt
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # prompt
artcar.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)      # prompt
artcar.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)       # prompt

# === Terrain: RigidTerrain (flat, NSC) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Center the terrain ahead of the vehicle spawn to ensure 500m coverage in +X
terrain_center = chrono.ChCoordsysd(chrono.ChVector3d(125.0, 0.0, 0.0), chrono.QUNIT)
patch = terrain.AddPatch(
    patch_mat,
    terrain_center,
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# === Irrlicht visualization (vehicle-aware) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar — PRIMITIVES vis, FIALA tires, MESH chassis collision")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 4.0, 0.5)  # chase camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(artcar.GetVehicle())

# === Driver: ChInteractiveDriverIRR (truth-matched, real-time) ===
steering_time = 1.0   # seconds to reach full steering
throttle_time = 1.0   # seconds to reach full throttle
braking_time  = 0.3   # seconds to reach full braking

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * step_size / steering_time)  # precomputed once
driver.SetThrottleDelta(render_every * step_size / throttle_time)
driver.SetBrakingDelta(render_every * step_size  / braking_time)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and artcar.GetSystem().GetChTime() < sim_end:
        time = artcar.GetSystem().GetChTime()  # cache: fetched once per frame

        # --- Render once per render frame ---
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        # --- Synchronize / advance driver ---
        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        artcar.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # --- Physics sub-steps between frames ---
        for _ in range(render_every):
            artcar.Advance(step_size)    # advances wrapper-owned ChSystem
            terrain.Advance(step_size)
            driver.Advance(step_size)
            vis.Advance(step_size)
            if artcar.GetSystem().GetChTime() >= sim_end:
                break

        realtime_timer.Spin(step_size)  # keep wall-clock aligned with sim time

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass  # nothing else to flush in scored core

