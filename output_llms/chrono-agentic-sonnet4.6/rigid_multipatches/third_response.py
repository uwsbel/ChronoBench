"""
Rigid Multi-Patch Terrain — HMMWV with four terrain patches at updated positions.

System type : NSC (RigidTerrain + NSC contact).
Vehicle     : HMMWV_Full on a multi-patch RigidTerrain composed of four patches:
              Patch 1 — flat box   at (-20, 5, 0)
              Patch 2 — mesh-based at (20, -5, 0.2)
              Patch 3 — BMP heightmap at (5, -45, 0)
              Patch 4 — BMP heightmap at (10, 40, 0)
Driver      : ChInteractiveDriverIRR (real-time keyboard control).
Expected    : Vehicle sits on the central patch; interactive steering/throttle.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as veh_irr
import pychrono.vehicle as veh

# === Constants ===
# --- Simulation timing ---
STEP_SIZE       = 2e-3            # physics step [s]
SIM_END         = 20.0            # simulation end time [s]; terminates recording run
RENDER_FPS      = 50.0            # rendering rate [Hz]
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# --- Vehicle init position (center patch area, wheel bottoms on z=0) ---
INIT_LOC        = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT        = chrono.ChQuaterniond(1, 0, 0, 0)

# --- Terrain patch geometry ---
PATCH_TEX_LEN   = 200.0
PATCH_TEX_WID   = 200.0

# --- Data paths (truth-required; bundled paths already resolve via defaults) ---
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                   # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()                         # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                    # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; joints: suspension+steering inside wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain (RigidTerrain — NSC; 4 patches at updated positions) ===
terrain = veh.RigidTerrain(sys)

# Contact material for all patches — NSC matches the vehicle's contact method
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 1: flat rectangular box at (-20, 5, 0)
patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-20, 5, 0), chrono.QUNIT),
    200.0, 200.0
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), PATCH_TEX_LEN, PATCH_TEX_WID)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: mesh-based at (20, -5, 0.2)
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(20, -5, 0.2), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj")
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), PATCH_TEX_LEN, PATCH_TEX_WID)
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))

# Patch 3: BMP heightmap at (5, -45, 0)
patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(5, -45, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    64.0, 64.0, 0.0, 3.0
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 6.0, 6.0)
patch3.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# Patch 4: BMP heightmap at (10, 40, 0)
patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    64.0, 64.0, 0.0, 3.0
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 6.0, 6.0)
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.8))

terrain.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht — vehicle-specific renderer) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Rigid Multi-Patch Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                           # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (ChInteractiveDriverIRR — scored-core default for catalog vehicles) ===
render_step_size = 1.0 / RENDER_FPS                # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


frame = 0

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


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
    pass
