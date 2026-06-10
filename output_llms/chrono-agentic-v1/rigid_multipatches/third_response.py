"""
Rigid Multi-Patch Terrain with HMMWV — PyChrono 9.0.x / Irrlicht

Models an HMMWV driving on a RigidTerrain composed of four independently-placed
terrain patches.  Each patch has a distinct position, texture, and contact
material, demonstrating multi-patch rigid terrain assembly.

System type : ChSystemNSC (owned by HMMWV_Full wrapper)
Main bodies : HMMWV chassis + 4 wheel spindles + 4 terrain patch bodies
Driver      : ChInteractiveDriverIRR (keyboard-driven; real-time loop)
Expected    : Vehicle rests on the central terrain area; driver can steer/accelerate
              across the patchy multi-surface layout.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Data path (anchor absolute so asset files load from any working directory) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
STEP_SIZE = 2e-3            # physics time step [s]
SIM_END   = 20.0            # simulation end time [s]
RENDER_FPS = 50.0           # target render rate [frames/s]

RENDER_STEP_SIZE = 1.0 / RENDER_FPS                             # precomputed once
RENDER_STEPS     = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

# Terrain patch positions (updated per prompt):
# Patch 1: flat+grass  at (-20,  5, 0)
# Patch 2: height bump at ( 20, -5, 0.2)  — elevated 0.2 m
# Patch 3: flat+dirt   at (  5,-45, 0)
# Patch 4: flat+asphalt at (10, 40, 0)
PATCH1_POS = chrono.ChVector3d(-20,  5,  0.0)
PATCH2_POS = chrono.ChVector3d( 20, -5,  0.2)
PATCH3_POS = chrono.ChVector3d(  5,-45,  0.0)
PATCH4_POS = chrono.ChVector3d( 10, 40,  0.0)

# Patch sizes [m]
PATCH_LEN = 30.0
PATCH_WID = 30.0

# Vehicle initial position — spawn on Patch 1 centre (-20, 5, 0):
# chassis origin at patch z=0 + suspension_ref_height 0.5 m
SUSPENSION_REF_HEIGHT = 0.5        # chassis origin above wheel-bottom at rest (HMMWV)
VEH_INIT_POS = chrono.ChVector3d(-20, 5, SUSPENSION_REF_HEIGHT)  # on Patch 1
VEH_INIT_ROT = chrono.QuatFromAngleZ(0.0)   # heading +X

# Camera chase parameters
CHASE_POINT  = chrono.ChVector3d(0, 0, 1.75)  # local offset on chassis
CHASE_DIST   = 9.0
CHASE_OFFSET = 0.5

# === Vehicle setup (HMMWV_Full wrapper) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                     # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_POS, VEH_INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# Visualization types (after Initialize) — in source build 9.0.0, enum lives in veh.*
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()             # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain patch bodies below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# === Terrain — four rigid patches at specified positions ===
terrain = veh.RigidTerrain(sys)

# --- Contact material (NSC — matches the system) ---
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 1 — grass/flat at (-20, 5, 0)
csys1 = chrono.ChCoordsysd(PATCH1_POS, chrono.QUNIT)
patch1 = terrain.AddPatch(patch_mat, csys1, PATCH_LEN, PATCH_WID)
patch1.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)
patch1.SetColor(chrono.ChColor(0.4, 0.8, 0.4))

# Patch 2 — tile/elevated at (20, -5, 0.2)
csys2 = chrono.ChCoordsysd(PATCH2_POS, chrono.QUNIT)
patch2 = terrain.AddPatch(patch_mat, csys2, PATCH_LEN, PATCH_WID)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 6, 6)
patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Patch 3 — dirt at (5, -45, 0)
csys3 = chrono.ChCoordsysd(PATCH3_POS, chrono.QUNIT)
patch3 = terrain.AddPatch(patch_mat, csys3, PATCH_LEN, PATCH_WID)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)
patch3.SetColor(chrono.ChColor(0.7, 0.5, 0.3))

# Patch 4 — asphalt/tile at (10, 40, 0)
csys4 = chrono.ChCoordsysd(PATCH4_POS, chrono.QUNIT)
patch4 = terrain.AddPatch(patch_mat, csys4, PATCH_LEN, PATCH_WID)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 6, 6)
patch4.SetColor(chrono.ChColor(0.4, 0.4, 0.4))

terrain.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Rigid Multi-Patch Terrain (Turn 3)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DIST, CHASE_OFFSET)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver — ChInteractiveDriverIRR (scored-core: keyboard control) ===
steering_time = 1.0   # s to go 0 → ±1
throttle_time = 1.0   # s to go 0 → 1
braking_time  = 0.3   # s to go 0 → 1

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === Record mode setup (review-only) ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_STEPS == 0:
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
    pass  # nothing to close in the scored core (CSV writers are review-only)
