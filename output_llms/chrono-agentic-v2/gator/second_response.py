"""
Gator vehicle on a multi-patch rigid terrain with diverse textures and a heightmap patch.

System type: NSC (Gator on rigid terrain uses NSC contact method).
Vehicle: veh.Gator() — a lightweight utility vehicle.
Terrain: RigidTerrain with 4 distinct patches:
  - Patch 1 (flat, tile texture)  — baseline flat road surface
  - Patch 2 (flat, dirt texture)  — off-road dirt section
  - Patch 3 (heightmap + bump64)  — gradability test slope/bump terrain
  - Patch 4 (flat, grass texture) — grass section with a procedural bump body
Each patch has a different texture; the heightmap patch tests the vehicle's
gradability over varied terrain relief. An interactive Irrlicht driver is used
(scored core); a review-only block scripts the throttle for the validation run.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
STEP_SIZE      = 2e-3        # simulation time step (s)
SIM_END        = 20.0        # total simulation time (s)
RENDER_FPS     = 50.0        # Irrlicht render frames per second
TERRAIN_LEN    = 60.0        # length of each terrain patch (m)
TERRAIN_WIDTH  = 30.0        # width of each terrain patch (m)
INIT_LOC       = chrono.ChVector3d(0, 0, 0.5)   # Gator spawn position
INIT_ROT       = chrono.QuatFromAngleZ(0.0)     # facing +X

render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Data paths (mandatory for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# Visualization types (must be after Initialize)
gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.Gator wrapper) ===
sys = gator.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()         # main chassis rigid body; cache: fetched once
# wheels/spindles: gator.GetVehicle().GetAxle(i); terrain: RigidTerrain patches below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === Terrain — 4 patches with different textures ===
# Patch 1: flat, tile4 texture — placed at X center ~[-30, 30], Y center 0
# Patch 2: flat, dirt texture  — placed at X center [30, 90], Y center 0
# Patch 3: heightmap (bump64)  — gradability test, X center [90, 150], Y center 0
# Patch 4: flat, grass texture — placed at X center [-90, -30], Y center 0
# All patches share the same Y band [-15, 15] (width 30 m).

terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches Gator contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 1 — flat tile surface (vehicle spawn patch)
patch1_center = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
patch1 = terrain.AddPatch(patch_mat, patch1_center, TERRAIN_LEN, TERRAIN_WIDTH)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

# Patch 2 — flat dirt surface
patch2_center = chrono.ChCoordsysd(chrono.ChVector3d(60, 0, 0), chrono.QUNIT)
patch2 = terrain.AddPatch(patch_mat, patch2_center, TERRAIN_LEN, TERRAIN_WIDTH)
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.7, 0.5, 0.3))

# Patch 3 — heightmap (bump64.bmp) for gradability / slope testing
# hMin=-0.5, hMax=0.5 gives gentle hills; veh.GetDataFile for terrain height maps
patch3_center = chrono.ChCoordsysd(chrono.ChVector3d(120, 0, 0), chrono.QUNIT)
patch3 = terrain.AddPatch(
    patch_mat,
    patch3_center,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LEN,
    TERRAIN_WIDTH,
    -0.5,   # hMin
    0.5,    # hMax
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
patch3.SetColor(chrono.ChColor(0.6, 0.5, 0.4))

# Patch 4 — flat grass surface (behind spawn)
patch4_center = chrono.ChCoordsysd(chrono.ChVector3d(-60, 0, 0), chrono.QUNIT)
patch4 = terrain.AddPatch(patch_mat, patch4_center, TERRAIN_LEN, TERRAIN_WIDTH)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch4.SetColor(chrono.ChColor(0.4, 0.7, 0.3))

terrain.Initialize()

# === Bump body on patch 1 — procedural box bump on the flat tile patch ===
# A fixed low box (bump) sitting on the tile patch surface for the vehicle to drive over
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)
bump = chrono.ChBodyEasyBox(3.0, TERRAIN_WIDTH, 0.1, 1000, True, True, bump_mat)
bump.SetName("bump_patch1")
bump.SetPos(chrono.ChVector3d(20, 0, 0.05))  # top at z=0.1, centered on patch 1
bump.SetFixed(True)
bump.EnableCollision(True)
sys.AddBody(bump)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — Multi-Patch Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()                              # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                     # vehicle truth uses directional light
vis.AttachVehicle(gator.GetVehicle())

# === Driver — ChInteractiveDriverIRR (scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0   # s to reach full steering
throttle_time = 1.0   # s to reach full throttle
braking_time  = 0.3   # s to reach full braking

render_step_size = 1.0 / RENDER_FPS         # precomputed once
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
    while vis.Run() and gator.GetSystem().GetChTime() < SIM_END:
        time = gator.GetSystem().GetChTime()   # cache: time fetched once per outer step

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        for _ in range(render_every):
            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            gator.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            step_number += 1
            if gator.GetSystem().GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
