"""
Gator vehicle on a 4-patch RigidTerrain with varied textures, one heightmap patch
(for gradability testing), and a bump obstacle patch.  The terrain is assembled from
four distinct patches placed side-by-side along the X-axis:
  Patch 0 — flat asphalt/tile texture (spawn pad)
  Patch 1 — flat dirt texture
  Patch 2 — heightmap bump64.bmp (gradability / hill terrain)
  Patch 3 — flat concrete texture with a raised box-bump obstacle

System type: NSC (rigid terrain, catalog Gator wrapper default).
Expected: Gator spawns on Patch 0, interactive driver steers across the varied
terrain including the heightmap hills and the bump obstacle on Patch 3.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants ===
step_size      = 2e-3          # simulation time step (s)
sim_end        = 30.0          # total simulation time (s)
render_fps     = 50.0
render_every   = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# Terrain patch layout: four patches arranged along +X axis
PATCH_LEN      = 40.0          # each patch length in X (m)
PATCH_WID      = 40.0          # each patch width in Y (m)
P0_X           = 0.0           # Patch 0 near edge X
P1_X           = PATCH_LEN     # Patch 1 near edge X
P2_X           = 2.0 * PATCH_LEN
P3_X           = 3.0 * PATCH_LEN

# Bump obstacle (on Patch 3)
BUMP_LEN       = 0.5
BUMP_WID       = PATCH_WID
BUMP_HEIGHT    = 0.12
BUMP_X         = P3_X + PATCH_LEN * 0.4  # 40% into patch 3

# Vehicle spawn
VEH_INIT_X     = P0_X + 5.0   # 5 m into Patch 0
VEH_INIT_Y     = 0.0
SUSPENSION_REF_HEIGHT = 0.30   # Gator chassis origin above wheel bottoms at rest

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)

init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, SUSPENSION_REF_HEIGHT)
init_rot = chrono.QuatFromAngleZ(0.0)
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(step_size)
gator.Initialize()

# === System & bodies (created by the veh.Gator wrapper) ===
system  = gator.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()               # cache: main chassis rigid body
# wheels/spindles: gator.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patches below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# Wheel-bottom check after Initialize
TIRE_RADIUS = 0.30   # approximate Gator tire radius
ZTOL = 0.10
veh_obj    = gator.GetVehicle()  # cache: fetched once
spindle_zs = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_zs.append(p.z)
wheel_bottom_z = min(spindle_zs) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Gator sinks into ground: wheel_bottom_z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Terrain — 4 patches with different textures, one heightmap, one with a bump ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 0 — flat tile/asphalt texture (spawn area)
csys0 = chrono.ChCoordsysd(
    chrono.ChVector3d(P0_X + PATCH_LEN * 0.5, 0.0, 0.0), chrono.QUNIT)
patch0 = terrain.AddPatch(patch_mat, csys0, PATCH_LEN, PATCH_WID)
patch0.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch0.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

# Patch 1 — flat dirt texture
csys1 = chrono.ChCoordsysd(
    chrono.ChVector3d(P1_X + PATCH_LEN * 0.5, 0.0, 0.0), chrono.QUNIT)
patch1 = terrain.AddPatch(patch_mat, csys1, PATCH_LEN, PATCH_WID)
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.7, 0.5, 0.3))

# Patch 2 — heightmap for gradability testing (bump64.bmp provides hills)
csys2 = chrono.ChCoordsysd(
    chrono.ChVector3d(P2_X + PATCH_LEN * 0.5, 0.0, 0.0), chrono.QUNIT)
patch2 = terrain.AddPatch(
    patch_mat,
    csys2,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    PATCH_LEN, PATCH_WID,
    -1.0, 1.0,     # height range [hMin, hMax] in metres
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.4, 0.6, 0.3))

# Patch 3 — flat concrete with a raised box bump obstacle
csys3 = chrono.ChCoordsysd(
    chrono.ChVector3d(P3_X + PATCH_LEN * 0.5, 0.0, 0.0), chrono.QUNIT)
patch3 = terrain.AddPatch(patch_mat, csys3, PATCH_LEN, PATCH_WID)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

terrain.Initialize()

# Bump obstacle — fixed box on Patch 3 (spans full width for maximum detectability)
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.8)
bump_mat.SetRestitution(0.01)
bump = chrono.ChBodyEasyBox(
    BUMP_LEN, BUMP_WID, BUMP_HEIGHT,
    2000.0, True, True, bump_mat,
)
bump.SetName("bump_obstacle")
bump.SetPos(chrono.ChVector3d(BUMP_X, 0.0, BUMP_HEIGHT * 0.5))
bump.SetFixed(True)
bump.EnableCollision(True)
system.AddBody(bump)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — 4-Patch Terrain with Heightmap & Bump")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(gator.GetVehicle())

# === Visualization types (set after Initialize) ===
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_NONE)

# === Driver — interactive IRR (scored core) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_every * step_size / steering_time)
driver.SetThrottleDelta(render_every * step_size / throttle_time)
driver.SetBrakingDelta(render_every * step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if frame % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        driver.Advance(step_size)


        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

        frame += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
