"""
PyChrono simulation: HMMWV on custom highway mesh terrain with multiple rigid patches.

System: NSC contact method, rigid terrain (highway mesh + bump mesh patches).
Bodies: HMMWV full vehicle (chassis, suspension, wheels, tires), terrain patches.
Expected behavior: HMMWV drives on the highway mesh terrain in real-time with
interactive driver control. A second bump patch (bump.obj) is positioned at
(6, -70, 0) with -90 degree Z-rotation, friction=0.4, restitution=0.05.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths (required for all catalog vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation constants ===
step_size = 2e-3            # physics time step (s)
sim_end = 20.0              # simulation duration (s)  -- review run end
render_fps = 50.0           # target render rate (Hz)
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

# === Vehicle initial conditions ===
# Highway mesh spans X:[-11.6,+11.6], Y:[-75,+75.8], Z:[0,0.78]
# Place vehicle on highway, raised above mesh surface to avoid initial overlap
INIT_LOC = chrono.ChVector3d(0.0, -60.0, 1.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# === Review-only setup (record flag + frame dir) ===

# === Vehicle setup (HMMWV Full) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)          # MANDATORY — fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # TMEASY required for proper traction
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === Visualization types (after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()              # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()        # cache: fetched once, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patches below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain (Rigid — highway mesh + bump patch) ===
terrain = veh.RigidTerrain(system)

# --- Contact material for highway mesh patch (original turn-1 patch) ---
highway_mat = chrono.ChContactMaterialNSC()
highway_mat.SetFriction(0.9)
highway_mat.SetRestitution(0.01)

# Add highway collision mesh patch (Highway_col.obj for physics contact)
highway_patch = terrain.AddPatch(
    highway_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj"),
)
highway_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
highway_patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200
)

# Add visual mesh overlay (Highway_vis.obj) as a visual shape on the collision patch body
highway_vis_shape = chrono.ChVisualShapeModelFile()
highway_vis_shape.SetFilename(chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"))
highway_patch.GetGroundBody().AddVisualShape(highway_vis_shape)

# --- Contact material for bump patch (turn3: friction=0.4, restitution=0.05) ---
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.4)           # updated from 0.9 → 0.4 (turn3)
patch_mat.SetRestitution(0.05)       # updated from 0.01 → 0.05 (turn3)

# Bump patch rotation: -90 degrees about Z-axis (turn3)
bump_rot = chrono.QuatFromAngleZ(-math.pi / 2.0)   # -90 deg about Z

# Add bump patch at (6, -70, 0) with -90 Z rotation (turn3 position update)
bump_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0), bump_rot),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.8))   # blue tint from turn2
bump_patch.SetTexture(
    veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0
)

terrain.Initialize()

# === Irrlicht visualization (vehicle-specific visual system) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()    # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — ChInteractiveDriverIRR) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0    # s from 0 to max steering
throttle_time = 1.0    # s from 0 to max throttle
braking_time = 0.3     # s from 0 to max braking

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === CSV logging setup (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
        time = hmmwv.GetSystem().GetChTime()

        # Throttled rendering (render at render_fps, not every physics step)
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order: driver -> terrain -> vehicle -> vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Log physics data (review-only)

        # Advance subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)    # advances the wrapper-owned ChSystem
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
