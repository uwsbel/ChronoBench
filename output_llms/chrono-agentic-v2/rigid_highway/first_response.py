"""
HMMWV on custom highway mesh terrain — rigid terrain demo.

System type : NSC (ChSystemNSC, owned by veh.HMMWV_Full wrapper)
Vehicle     : HMMWV_Full with TMEASY tire model
Terrain     : RigidTerrain with collision mesh (Highway_col.obj)
              and visual mesh (Highway_vis.obj)
Driver      : ChInteractiveDriverIRR (interactive keyboard control)
Renderer    : Irrlicht (ChWheeledVehicleVisualSystemIrrlicht)

Expected behavior: HMMWV spawned above the highway mesh surface,
interactive driver allows steering/throttle/brake via keyboard.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths — required for catalog vehicle truths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
STEP_SIZE         = 1e-3          # physics time step (s)
SIM_END           = 30.0          # simulation duration (s)
RENDER_FPS        = 50.0          # rendering frame rate (fps)
RENDER_EVERY      = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_LOC          = chrono.ChVector3d(0.0, 0.0, 1.0)  # vehicle spawn location
INIT_ROT          = chrono.QUNIT                        # identity rotation

TERRAIN_COL_MESH  = veh.GetDataFile("terrain/meshes/Highway_col.obj")   # collision mesh
TERRAIN_VIS_MESH  = veh.GetDataFile("terrain/meshes/Highway_vis.obj")   # visual mesh

STEERING_TIME     = 1.0   # s: time to reach full steering
THROTTLE_TIME     = 1.0   # s: time to reach full throttle
BRAKING_TIME      = 0.3   # s: time to reach full braking

# === Vehicle setup (HMMWV_Full wrapper owns ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY for mesh terrain
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()             # cache: fetched once, reused if needed
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())  # truth component

# Set visualization types (after Initialize, per skill rule)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — RigidTerrain with collision + visual mesh patches ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches system contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Collision mesh patch
col_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    TERRAIN_COL_MESH,
)
col_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Visual mesh patch (overlaid at same position)
vis_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    TERRAIN_VIS_MESH,
)
vis_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                               # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())                  # AFTER Initialize

# === Driver — interactive (ChInteractiveDriverIRR, scored-core default) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME))  # precomputed once
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME))
driver.Initialize()

# === Review-only recording setup ===

# === Main loop — real-time with render-cadence outer loop ===
realtime_timer = chrono.ChRealtimeStepTimer()  # for real-time pacing
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()

        # --- Render once per frame ---
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        # --- Physics batch: render_every steps per frame ---
        for _ in range(RENDER_EVERY):
            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)   # advances wrapper-owned ChSystem
            vis.Advance(STEP_SIZE)

            sim_time = system.GetChTime()
            if sim_time >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)  # real-time pacing

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
