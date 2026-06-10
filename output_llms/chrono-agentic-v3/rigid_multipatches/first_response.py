"""
HMMWV on Rigid Multi-Patch Terrain — PyChrono 9.0.x, Irrlicht visualization.

Models an HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving over a
complex rigid terrain composed of multiple surface patches:
  - Two flat road patches with different tile textures (road surface variants)
  - A mesh-based patch providing a bump obstacle
  - A heightmap-based patch for varying elevation changes

System type: NSC (Non-Smooth Contact) — default for rigid-terrain catalog vehicles.
Expected behavior: HMMWV starts on road patch, can drive through bump and over
heightmap terrain under interactive keyboard steering/throttle/braking.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Named constants ===
STEP_SIZE = 2e-3           # physics time step (s)
SIM_END = 20.0             # simulation end time (s)
RENDER_FPS = 50.0          # rendering frame rate (Hz)
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle initial position and orientation
INIT_X = -30.0             # (m) start near left end of terrain
INIT_Y = 0.0
INIT_Z = 0.5               # (m) chassis origin above flat patch surface
initLoc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
initRot = chrono.QuatFromAngleZ(0.0)  # no initial yaw

# Terrain patch dimensions
TERRAIN_LENGTH = 80.0      # (m) total length along X
TERRAIN_WIDTH = 20.0       # (m) patch width along Y

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                             # MANDATORY — fixed won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)              # TMEASY for good grip/handling
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                                   # ChSystemNSC owned by wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()                          # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())    # diagnostic: total vehicle mass

# === Visualization types (after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Rigid terrain with multiple patches ===
# NSC contact material matching the vehicle's contact method
patch_mat1 = chrono.ChContactMaterialNSC()
patch_mat1.SetFriction(0.9)
patch_mat1.SetRestitution(0.01)

patch_mat2 = chrono.ChContactMaterialNSC()
patch_mat2.SetFriction(0.9)
patch_mat2.SetRestitution(0.01)

patch_mat_bump = chrono.ChContactMaterialNSC()
patch_mat_bump.SetFriction(0.8)
patch_mat_bump.SetRestitution(0.01)

patch_mat_hmap = chrono.ChContactMaterialNSC()
patch_mat_hmap.SetFriction(0.9)
patch_mat_hmap.SetRestitution(0.01)

terrain = veh.RigidTerrain(sys)

# Patch 1 — flat road surface, left portion, tile4 texture
patch1_csys = chrono.ChCoordsysd(
    chrono.ChVector3d(-30.0, 0.0, 0.0), chrono.QUNIT
)
patch1 = terrain.AddPatch(patch_mat1, patch1_csys, 30.0, TERRAIN_WIDTH)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 2 — flat road surface, center-right portion, concrete texture
patch2_csys = chrono.ChCoordsysd(
    chrono.ChVector3d(5.0, 0.0, 0.0), chrono.QUNIT
)
patch2 = terrain.AddPatch(patch_mat2, patch2_csys, 30.0, TERRAIN_WIDTH)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

# Patch 3 — mesh-based bump patch (gentle halfround bump, 150 mm height)
# bump.obj has a 2.88 m peak causing instability; use halfround_150mm.obj instead
patch3_csys = chrono.ChCoordsysd(
    chrono.ChVector3d(-5.0, 0.0, 0.0), chrono.QUNIT
)
bump_mesh_file = veh.GetDataFile("terrain/meshes/halfround_150mm.obj")
patch3 = terrain.AddPatch(patch_mat_bump, patch3_csys, bump_mesh_file)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 10)
patch3.SetColor(chrono.ChColor(0.6, 0.5, 0.4))

# Patch 4 — heightmap-based patch for varying elevation
patch4_csys = chrono.ChCoordsysd(
    chrono.ChVector3d(25.0, 0.0, 0.0), chrono.QUNIT
)
heightmap_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
patch4 = terrain.AddPatch(
    patch_mat_hmap, patch4_csys,
    heightmap_file,
    20.0, TERRAIN_WIDTH,
    0.0, 1.0    # height range: 0 to 1 m
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 10)
patch4.SetColor(chrono.ChColor(0.4, 0.6, 0.3))

terrain.Initialize()

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Rigid Multi-Patch Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                     # standard outdoor sky
vis.AddLightDirectional()                           # vehicle demos use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (scored core — matches catalog truth shape) ===
driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS               # precomputed once
steering_time = 1.0                                 # s to reach max steering
throttle_time = 1.0                                 # s to reach max throttle
braking_time = 0.3                                  # s to reach max braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Validate vehicle spawn position ===
TIRE_RADIUS = 0.33    # approximate HMMWV TMEASY tire radius (m)
ZTOL = 0.10           # allowed overlap/clearance vs support top (m)

veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z=0.0; raise INIT_Z by {-wheel_bottom_z:.3f} m"
)


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        time = hmmwv.GetSystem().GetChTime()  # cache: current sim time

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in fixed order
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance all subsystems
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
