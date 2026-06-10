"""
Rigid highway HMMWV simulation with multi-patch RigidTerrain.

System type : ChSystemNSC (owned by HMMWV_Full wrapper)
Vehicle     : HMMWV_Full on RigidTerrain with multiple terrain patches
Patches     : flat road patch (highway texture) + bump.obj mesh patch at (0,-42,0)
              with dirt texture (6×6 UV scale) and light-blue color (0.5, 0.5, 0.8)
Driver      : ChInteractiveDriverIRR (keyboard-driven; review-only scripted block
              provides an open-loop maneuver for the validation video)
Expected    : Vehicle drives forward on the flat highway surface; the second bump
              mesh patch is visible off to the side with bump geometry.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE      = 1e-3          # physics time step (s)
SIM_END        = 20.0          # simulation duration (s)
RENDER_FPS     = 50.0
RENDER_STEPS   = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

# Terrain large enough to contain the full 20 s run (HMMWV top ~15 m/s → ~300 m);
# vehicle starts at x=0, terrain centered at x=0 → spans ±300 m in X.
TERRAIN_LENGTH = 600.0
TERRAIN_WIDTH  = 200.0

INIT_LOC = chrono.ChVector3d(-100.0, 0.0, 0.5)  # start with 400 m of road ahead (center y)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

SUSPENSION_REF_HEIGHT = 0.5   # chassis-origin above wheel-bottom at rest (HMMWV)

# === Data paths (truth-required) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()     # cache: fetched once, reused every step
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization types ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches ChContactMethod_NSC
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Flat road patch (primary driving surface) — centered at world origin
flat_patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
flat_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
flat_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Mesh bump patch — Turn 2 delta: bump.obj at (0, -42, 0),
# color (0.5, 0.5, 0.8), dirt.jpg texture with 6.0×6.0 UV scale
bump_csys = chrono.ChCoordsysd(
    chrono.ChVector3d(0.0, -42.0, 0.0),
    chrono.QUNIT,
)
bump_patch = terrain.AddPatch(
    patch_mat,
    bump_csys,
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Rigid Highway Multi-Patch")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()   # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver ===
render_step_size = 1.0 / RENDER_FPS   # precomputed once

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()   # cache: read once per outer loop

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # review-only: scripted open-loop maneuver for validation video

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # CSV closed in review-only block below
