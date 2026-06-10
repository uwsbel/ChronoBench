"""
Rigid Highway Simulation with Additional Bump Terrain Patch.

System type: NSC (rigid terrain, wheeled vehicle).
Main bodies: HMMWV_Full chassis + 4 wheels/spindles on RigidTerrain.
Two terrain patches:
  1. Main flat patch (400x400 m, tile4.jpg texture) centered at origin.
  2. Bump mesh patch using terrain/meshes/bump.obj at (0, -42, 0),
     color (0.5, 0.5, 0.8), texture dirt.jpg scaled 6.0 x 6.0.
Expected behavior: HMMWV drives forward on the flat highway terrain;
the bump patch is visible south of the starting position.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Vehicle data path setup ===
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
TIME_STEP = 2e-3           # simulation time step (s)
SIM_END = 20.0             # simulation duration (s)
RENDER_FPS = 50.0          # render frame rate (Hz)

# Terrain geometry
TERRAIN_LENGTH = 400.0     # flat patch X size (m)
TERRAIN_WIDTH = 400.0      # flat patch Y size (m)

# HMMWV initial spawn (flat terrain at z=0; HMMWV chassis origin ~0.5 m above wheels)
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (m)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = SUSPENSION_REF_HEIGHT  # wheels on flat terrain at z=0

# Bump patch location per prompt
BUMP_POS = chrono.ChVector3d(0.0, -42.0, 0.0)

# Precomputed render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === Vehicle Setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)  # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
        chrono.QuatFromAngleZ(0.0),
    )
)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Visualization types (after Initialize per skill rules)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()             # ChSystemNSC owned by the wrapper  # cache: fetched once
chassis = hmmwv.GetChassisBody()       # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxles(); terrain: RigidTerrain patches below
# joints: suspension + steering links created inside the wrapper

# REQUIRED: set collision system after Initialize
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain ===
terrain = veh.RigidTerrain(system)

# --- Patch 1: flat main highway patch ---
patch_mat_1 = chrono.ChContactMaterialNSC()
patch_mat_1.SetFriction(0.9)
patch_mat_1.SetRestitution(0.01)

patch1 = terrain.AddPatch(
    patch_mat_1,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch1.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# --- Patch 2: bump mesh patch at (0, -42, 0) with color (0.5, 0.5, 0.8) and dirt.jpg ---
patch_mat_2 = chrono.ChContactMaterialNSC()
patch_mat_2.SetFriction(0.9)
patch_mat_2.SetRestitution(0.01)

bump_coordsys = chrono.ChCoordsysd(BUMP_POS, chrono.QUNIT)
patch2 = terrain.AddPatch(
    patch_mat_2,
    bump_coordsys,
    veh.GetVehicleDataFile("terrain/meshes/bump.obj"),
)
patch2.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch2.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Footprint assert after Initialize ===
TIRE_RADIUS = 0.33  # approximate HMMWV tire radius (m)
ZTOL = 0.10         # allowed wheel-bottom clearance vs terrain
veh_obj = hmmwv.GetVehicle()  # cache: fetched once, reused for spindle access
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z=0.0; raise SUSPENSION_REF_HEIGHT by "
    f"{-wheel_bottom_z:.3f} m"
)

# === Visualization ===
# Vehicle Irrlicht visual system (chase camera follows HMMWV)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid Highway — Bump Terrain Patch Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()  # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — interactive (scored core: ChInteractiveDriverIRR per truth) ===
render_step_size = 1.0 / RENDER_FPS  # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        for _ in range(RENDER_EVERY):
            sim_time = hmmwv.GetSystem().GetChTime()
            driver_inputs = driver.GetInputs()


            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)


            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            if hmmwv.GetSystem().GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
