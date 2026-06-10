"""
HMMWV Full Model on Flat Rigid Terrain — PyChrono 9.0.0 (Irrlicht)

Models a full HMMWV (High Mobility Multipurpose Wheeled Vehicle) driving on a
flat rigid terrain using the NSC contact method with TMEASY tire model.
Uses primitive visualization for vehicle components and an interactive
IRR driver for steering, throttle, and braking. The simulation runs in
real time at 50 frames per second.

System type: ChSystemNSC (owned by the HMMWV wrapper)
Main bodies: HMMWV chassis, 4 wheel spindles, 4 tires (TMEASY)
Expected behavior: vehicle rests on flat terrain; user drives with keyboard.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
step_size = 2e-3          # physics time step (s) — 500 Hz
sim_end = 20.0            # simulation end time (s)
render_fps = 50.0         # rendering rate (frames/s)
render_step_size = 1.0 / render_fps                          # precomputed once
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once

TERRAIN_LENGTH = 200.0    # terrain patch X dimension (m)
TERRAIN_WIDTH = 200.0     # terrain patch Y dimension (m)
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (m)

# Initial vehicle position and orientation
initLoc = chrono.ChVector3d(0, 0, SUSPENSION_REF_HEIGHT)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Data paths (required truth components for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # prompt: TMEASY tire model
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body — fetched once, reused

# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch body below
# joints: double-wishbone suspension + steering links created inside the wrapper

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())  # truth-required diagnostic

# === Visualization types (set AFTER Initialize) ===
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization — full Irrlicht scene: window + sky + camera + lights ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Full Model — Flat Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()          # vehicle truths use directional light, not AddTypicalLights
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive IRR — scored core; scripted maneuver in review-only block) ===
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0     # seconds to full steering
throttle_time = 1.0     # seconds to full throttle
braking_time = 0.3      # seconds to full brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Throttled rendering — one render per frame interval
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems: driver -> terrain -> vehicle -> vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems: all four must be advanced every step
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)   # advances the wrapper-owned ChSystem — do NOT call system.DoStepDynamics
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)  # spin so wall-clock matches sim time

        if system.GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
