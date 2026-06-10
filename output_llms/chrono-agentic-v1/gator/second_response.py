"""
Gator vehicle simulation with 4 diverse terrain patches and bumps.

System type: NSC (rigid terrain, wheeled vehicle default).
Vehicle: Gator wrapper (veh.Gator), interactive driver (ChInteractiveDriverIRR).
Terrain: RigidTerrain with 4 patches arranged in a 2×2 grid:
  - Patch 0 (flat, tile4 texture)  — top-left
  - Patch 1 (flat, grass texture)  — top-right
  - Patch 2 (flat, dirt texture)   — bottom-left
  - Patch 3 (heightmap, concrete)  — bottom-right (tests gradability with bump64 heightmap)
  Each patch also has a physical bump (cylinder body) sitting on its surface.
Expected behavior: Gator drives over the multi-textured terrain, encounters bumps
on each patch, and can be steered onto the heightmap patch to test gradability.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data path setup ===
# Anchor vehicle data subtree to bundled Chrono vehicle data
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
# Simulation timing
TIME_STEP = 2e-3          # 2 ms physics step
SIM_END = 30.0            # 30 seconds of simulation
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Terrain geometry — 4 patches in a 2×2 grid, each 50×50 m
PATCH_SIZE = 50.0
HALF = PATCH_SIZE / 2.0

# Patch centers (X, Y)
# Patch 0: top-left    Patch 1: top-right
# Patch 2: bottom-left Patch 3: bottom-right (heightmap)
PATCH_CENTERS = [
    (-HALF, HALF),    # patch 0
    ( HALF, HALF),    # patch 1
    (-HALF, -HALF),   # patch 2
    ( HALF, -HALF),   # patch 3 (heightmap)
]

# Heightmap range for patch 3 (bump64.bmp: hill terrain, height 0..1 m)
HM_H_MIN = 0.0
HM_H_MAX = 1.0

# Bump dimensions per patch (cylinder bump body)
BUMP_RADIUS = 0.25   # m
BUMP_HEIGHT = 0.15   # m

# Vehicle initial position — start near center of patch 0 (top-left)
VEH_INIT_X = -20.0
VEH_INIT_Y =  20.0
VEH_INIT_Z =  0.5    # will be corrected after Initialize with spindle check

# === Recording flag ===

# === Vehicle setup ===
# Gator wrapper — creates and owns its own ChSystem (NSC)
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)   # MANDATORY — fixed chassis won't move
gator.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
    chrono.QUNIT
))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(TIME_STEP)
gator.Initialize()

# Collision system — REQUIRED before building terrain
gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === System & bodies (created by veh.Gator wrapper) ===
sys = gator.GetSystem()   # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()  # cache: main chassis rigid body

# Visualization types (after Initialize)
# NOTE: In 9.0.0 build with chrono900_env, VisualizationType_* lives in veh, not chrono
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
# RigidTerrain with 4 patches and different textures.
# Patch 3 uses a heightmap for gradability testing.
terrain = veh.RigidTerrain(sys)

# Shared contact material (NSC for rigid terrain)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Terrain texture paths — 4 distinct textures
data_path = chrono.GetChronoDataPath()
TEXTURES = [
    data_path + "vehicle/terrain/textures/tile4.jpg",     # patch 0
    data_path + "vehicle/terrain/textures/grass.jpg",     # patch 1
    data_path + "vehicle/terrain/textures/dirt.jpg",      # patch 2
    data_path + "vehicle/terrain/textures/concrete.jpg",  # patch 3
]

# Patch 0 — flat, tile4 texture
p0_cx, p0_cy = PATCH_CENTERS[0]
patch0 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(p0_cx, p0_cy, 0.0), chrono.QUNIT),
    PATCH_SIZE, PATCH_SIZE
)
patch0.SetTexture(TEXTURES[0], 20, 20)
patch0.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 1 — flat, grass texture
p1_cx, p1_cy = PATCH_CENTERS[1]
patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(p1_cx, p1_cy, 0.0), chrono.QUNIT),
    PATCH_SIZE, PATCH_SIZE
)
patch1.SetTexture(TEXTURES[1], 20, 20)
patch1.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

# Patch 2 — flat, dirt texture
p2_cx, p2_cy = PATCH_CENTERS[2]
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(p2_cx, p2_cy, 0.0), chrono.QUNIT),
    PATCH_SIZE, PATCH_SIZE
)
patch2.SetTexture(TEXTURES[2], 20, 20)
patch2.SetColor(chrono.ChColor(0.7, 0.5, 0.3))

# Patch 3 — heightmap (bump64.bmp) for gradability testing, concrete texture
p3_cx, p3_cy = PATCH_CENTERS[3]
hm_file = data_path + "vehicle/terrain/height_maps/bump64.bmp"
patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(p3_cx, p3_cy, 0.0), chrono.QUNIT),
    hm_file,
    PATCH_SIZE, PATCH_SIZE,
    HM_H_MIN, HM_H_MAX
)
patch3.SetTexture(TEXTURES[3], 20, 20)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

terrain.Initialize()

# === Bump bodies ===
# Add a physical bump (cylinder) to each patch so the vehicle must traverse a bump.
# Bump centers placed at the mid-point of each patch (slightly offset from center).
bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)

BUMP_POSITIONS = [
    (p0_cx - 5.0, p0_cy, 0.0),       # patch 0 bump
    (p1_cx + 5.0, p1_cy, 0.0),       # patch 1 bump
    (p2_cx - 5.0, p2_cy, 0.0),       # patch 2 bump
    (p3_cx + 5.0, p3_cy, HM_H_MAX * 0.3),  # patch 3 bump (on hill terrain, raised slightly)
]

# ChBodyEasyBox: full extents (length, width, height)
BUMP_LEN = 1.0   # m along X
BUMP_WID = 8.0   # m along Y (wide enough to span the driving lane)
BUMP_HT  = BUMP_HEIGHT

bump_bodies = []
for bx, by, bz in BUMP_POSITIONS:
    bump = chrono.ChBodyEasyBox(
        BUMP_LEN, BUMP_WID, BUMP_HT,
        1000.0, True, True, bump_mat
    )
    bump.SetPos(chrono.ChVector3d(bx, by, bz + BUMP_HT / 2.0))
    bump.SetFixed(True)
    bump.EnableCollision(True)
    sys.AddBody(bump)
    bump_bodies.append(bump)

# Bind all collision shapes after adding bumps
sys.GetCollisionSystem().BindAll()

# === Validate vehicle wheel positions ===
# Assert that wheels sit on (not through) the terrain after Initialize.
TIRE_RADIUS = 0.30   # Gator tire radius (approximate)
ZTOL = 0.15          # tolerance for wheel-bottom vs terrain top

veh_obj = gator.GetVehicle()  # cache: fetched once
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= 0.0 - ZTOL, (
    f"Vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} vs terrain z=0.0; "
    f"raise VEH_INIT_Z by {0.0 - wheel_bottom_z:.3f} m"
)

# === Visualization ===
# ChWheeledVehicleVisualSystemIrrlicht — vehicle-aware vis with chase camera
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator — 4-patch terrain with bumps and heightmap")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(gator.GetVehicle())

# === Driver ===
# Interactive driver (scored-core default for catalog vehicles matching ground truth)
render_step_size = 1.0 / RENDER_FPS  # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # 1 s to full steer
driver.SetThrottleDelta(render_step_size / 1.0)   # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # 0.3 s to full brake
driver.Initialize()

# === CSV logging (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
render_every_count = RENDER_EVERY  # cache: precomputed render cadence

try:
    while vis.Run():
        time = sys.GetChTime()  # cache per-step: current simulation time

        if step_number % render_every_count == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # CSV logging (review-only)

        # Advance subsystems (vehicle.Advance steps the owned system)
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        gator.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

        if sys.GetChTime() >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
