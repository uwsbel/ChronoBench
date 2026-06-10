"""
HMMWV (High Mobility Multipurpose Wheeled Vehicle) simulation on flat rigid terrain.

System type: NSC (ChSystemNSC owned by HMMWV_Full wrapper)
Main bodies: HMMWV chassis, 4 wheel spindles, rigid terrain patch
Tire model: TMEASY (prompt-specified)
Visualization: primitive shapes for vehicle components via Irrlicht
Expected behavior: HMMWV rests on flat rigid terrain; interactive driver allows
real-time steering, throttle, and braking control at 50 FPS.
"""

# === Imports ===
import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr  # noqa: F401 (used via vis below)


# === Constants ===
# Simulation timing
TIME_STEP = 2e-3          # 2 ms physics step (standard for rigid-terrain HMMWV)
SIM_END = 30.0            # seconds; enough for an interactive demo
RENDER_FPS = 50.0         # prompt-specified: 50 fps
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # steps between renders; precomputed once

# Terrain dimensions
TERRAIN_LENGTH = 200.0    # meters in X
TERRAIN_WIDTH = 200.0     # meters in Y

# Vehicle initial position (flat terrain at z=0, HMMWV suspension ref ~0.5 m)
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest
INIT_Z = 0.0 + SUSPENSION_REF_HEIGHT
INIT_POS = chrono.ChVector3d(0.0, 0.0, INIT_Z)
INIT_ROT = chrono.QuatFromAngleZ(0.0)  # facing +X

# Interactive driver ramp times
STEERING_TIME = 1.0   # seconds to full steering
THROTTLE_TIME = 1.0   # seconds to full throttle
BRAKING_TIME = 0.3    # seconds to full brake

# === Vehicle setup (HMMWV_Full wrapper — owns its own ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain (truth)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                              # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                     # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

# REQUIRED: collision system for contact with terrain
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization types (primitive per prompt) — set after Initialize ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)     # prompt: primitive; VisualizationType lives in veh namespace here
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()    # NSC matches the vehicle contact method
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

# === Validate vehicle footprint after Initialize ===
TIRE_RADIUS = 0.33    # HMMWV TMEASY tire radius (approximate)
ZTOL = 0.10           # generous tolerance for spindle Z check

veh_obj = hmmwv.GetVehicle()   # cache: fetched once, reused every step
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Irrlicht visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Flat Rigid Terrain — TMEASY Tires")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver — ChInteractiveDriverIRR (truth form) ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / (RENDER_FPS * STEERING_TIME))   # precomputed once
driver.SetThrottleDelta(1.0 / (RENDER_FPS * THROTTLE_TIME))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * BRAKING_TIME))
driver.Initialize()

# === Record-mode setup (review-only) ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)     # internally calls sys.DoStepDynamics — do NOT call it again
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
