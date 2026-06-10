"""
HMMWV on Rigid Multi-Patch Terrain — PyChrono 9.0.x / Irrlicht

Simulates a High Mobility Multipurpose Wheeled Vehicle (HMMWV) navigating a
complex rigid terrain composed of multiple patches of diverse surface types:
  - Flat patch 1: tile texture (road-like)
  - Flat patch 2: concrete texture (raised slightly)
  - Heightmap patch: bump64 elevation map with grass texture
  - Mesh patch: bump.obj geometry with dirt texture

The vehicle is initialized with SHAFTS engine, SHAFTS automatic transmission,
and all-wheel drive (AWD). All vehicle components use MESH visualization.
An interactive IRR driver (keyboard/gamepad) controls steering, throttle, and braking.
The simulation runs with a 2 ms timestep, rendered at 50 fps real-time.

System type: ChSystemNSC (owned by HMMWV wrapper), NSC contact materials.
Contact: Bullet collision system required for terrain-wheel contact.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Data path (must be absolute so files resolve correctly from any cwd) ===
# veh.GetDataPath() returns a relative path by default; anchor it using the
# Chrono data root so mesh assets load correctly regardless of working directory.
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
STEP_SIZE = 2e-3          # physics timestep (s)
SIM_END = 30.0            # simulation duration (s)
RENDER_FPS = 50.0         # render frame rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Vehicle init position ===
# Place HMMWV at the origin of patch 1 (-15, 0, 0 center, 60×20 m).
# HMMWV chassis origin is geometric center; suspension_ref_height ≈ 0.5 m.
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest
INIT_X = -25.0                # well within patch 1 x range [-45, 15]
INIT_Y = 0.0
INIT_Z = SUSPENSION_REF_HEIGHT  # terrain is at z=0 for patch 1


# === Vehicle setup (HMMWV_Full wrapper) ===
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)   # facing +X

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)      # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)   # no extra chassis collision
hmmwv.SetChassisFixed(False)                            # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
# Prompt specifies: SHAFTS engine, SHAFTS automatic transmission
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)             # prompt: AWD drivetrain
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY for reliable rigid-terrain contact
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()           # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()     # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created internally by the wrapper

# Collision system — REQUIRED for terrain-wheel contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization types — MESH for all components (prompt requirement) ===
# Note: in this 9.0.0 build VisualizationType_* lives in veh.*, not chrono.*
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Validate wheel placement after Initialize ===
TIRE_RADIUS = 0.33          # approximate HMMWV tire radius
ZTOL = 0.15                 # allowed clearance

veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks too deep: wheel_bottom_z={wheel_bottom_z:.3f} m. "
    f"Raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Terrain — RigidTerrain with multiple patches ===
terrain = veh.RigidTerrain(system)

# Shared NSC contact material for all patches
def make_patch_mat():
    """Create a reusable NSC contact material for terrain patches."""
    m = chrono.ChContactMaterialNSC()
    m.SetFriction(0.9)
    m.SetRestitution(0.01)
    return m

# Patch 1: flat box — tile texture (road-like surface)
mat1 = make_patch_mat()
patch1 = terrain.AddPatch(
    mat1,
    chrono.ChCoordsysd(chrono.ChVector3d(-15.0, 0.0, 0.0), chrono.QUNIT),
    60.0, 20.0   # length × width (m)
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 60, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2: flat box — concrete texture (slightly elevated, different surface)
mat2 = make_patch_mat()
patch2 = terrain.AddPatch(
    mat2,
    chrono.ChCoordsysd(chrono.ChVector3d(20.0, 0.0, 0.1), chrono.QUNIT),
    20.0, 30.0
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 30)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

# Patch 3: heightmap-based patch — bump64 elevation with grass texture
mat3 = make_patch_mat()
patch3 = terrain.AddPatch(
    mat3,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 42.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    64.0, 64.0,    # length × width
    0.0, 3.0       # height range [hMin, hMax]
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64, 64)
patch3.SetColor(chrono.ChColor(0.4, 0.8, 0.4))

# Patch 4: mesh-based patch — bump.obj geometry with dirt texture
mat4 = make_patch_mat()
patch4 = terrain.AddPatch(
    mat4,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -42.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj")
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.5, 0.4, 0.3))

terrain.Initialize()

# === Irrlicht visualization (vehicle visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Rigid Multi-Patch Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()                             # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (IRR — keyboard / gamepad) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0    # s to reach max steering
throttle_time = 1.0    # s to reach max throttle
braking_time  = 0.3    # s to reach max braking
render_step   = 1.0 / RENDER_FPS

driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step / braking_time)
driver.Initialize()

# === CSV logging setup ===

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run():
        time = system.GetChTime()

        # Render at throttled cadence (50 Hz)
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize: driver → terrain → vehicle → vis (required order)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance: driver → terrain → vehicle (steps the system) → vis
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)    # vehicle.Advance steps the owned system
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)    # pace wall-clock to sim time

        if system.GetChTime() >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
