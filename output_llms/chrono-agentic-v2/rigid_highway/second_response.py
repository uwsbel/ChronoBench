"""
rigid_highway simulation — HMMWV on a multi-patch RigidTerrain highway scene.

System type : NSC (ChSystemNSC owned by the HMMWV_Full wrapper)
Main bodies : HMMWV chassis, 4 wheel spindles, multi-patch rigid terrain
              (flat highway patches + mesh bump patch at (0, -42, 0))
Expected    : HMMWV drives forward on a rigid road with an additional mesh
              bump patch (bump.obj) to the side at (0, -42, 0), colored
              (0.5, 0.5, 0.8) with a dirt.jpg texture scaled 6 × 6.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths (truth-required for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
step_size        = 1e-3          # physics time step (s)
sim_end          = 20.0          # simulation duration (s)
render_fps       = 50.0          # target render rate
render_every     = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once
terrain_length   = 200.0         # terrain patch length (m)
terrain_width    = 100.0         # terrain patch width (m)

# Initial vehicle pose
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Vehicle (HMMWV_Full wrapper owns ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)    # MANDATORY — fixed chassis never moves
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # cache: fetched once, reused later
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patches below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Set visualization types
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (RigidTerrain with multiple patches) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()  # NSC to match the vehicle contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Primary flat highway patch
patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Additional bump mesh patch at (0, -42, 0) — bump.obj with dirt texture
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
patch2.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization (veh.ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Rigid Highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — truth uses ChInteractiveDriverIRR) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / render_fps   # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
        sim_time = hmmwv.GetSystem().GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)


        step_number += 1
        realtime_timer.Spin(step_size)

        if hmmwv.GetSystem().GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
