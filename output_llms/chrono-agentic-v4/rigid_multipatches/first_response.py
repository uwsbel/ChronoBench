"""
PyChrono simulation: HMMWV on multi-patch rigid terrain with Irrlicht visualization.

plan_type: mbs_in_scene (wrapper-managed vehicle)
system:     ChSystemNSC (rigid terrain, NSC contacts)
vehicle:    HMMWV_Full with TMEASY tires
terrain:    RigidTerrain with flat textured, mesh-bump, and heightmap patches
driver:     ChInteractiveDriverIRR (interactive keyboard control)
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Time / physics constants ===
time_step = 1e-3          # physics timestep [s]
sim_end = 30.0            # simulation duration [s]
render_fps = 50.0         # rendering frames per second
render_step_size = 1.0 / render_fps
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Vehicle init pose ===
# Terrain top near z=0; HMMWV suspension ref height ~0.5 m
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT

# === Create HMMWV ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Cache handles to essential wrapper-created bodies
chassis = hmmwv.GetChassisBody()  # cache: main chassis rigid body

# === Apply mesh visualization to vehicle components ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === RigidTerrain: three patches ===
terrain = veh.RigidTerrain(system)

# Patch 1 — flat, large, tiled texture
patch_mat_flat = chrono.ChContactMaterialNSC()
patch_mat_flat.SetFriction(0.8)
patch_mat_flat.SetRestitution(0.01)

patch_flat = terrain.AddPatch(
    patch_mat_flat,
    chrono.CSYSNORM,
    60.0,   # length X
    60.0,   # width Y
)
patch_flat.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

# Patch 2 — mesh-based bump in a sub-region (x: 2–6, y: –2–2)
# Load the bump mesh from Chrono's data files
bump_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/bump.obj")
bump_patch_mat = chrono.ChContactMaterialNSC()
bump_patch_mat.SetFriction(0.8)
bump_patch_mat.SetRestitution(0.01)

# Place patch 2 centred at (4, 0) with no rotation
bump_cs = chrono.ChCoordsysd(
    chrono.ChVector3d(4.0, 0.0, 0.0),
    chrono.QUNIT,
)
terrain.AddPatch(bump_patch_mat, bump_cs, bump_mesh_file)

# Patch 3 — heightmap-based elevation patch (placed at x: –10 to –4)
hm_mat = chrono.ChContactMaterialNSC()
hm_mat.SetFriction(0.8)
hm_mat.SetRestitution(0.01)

# Use the shipped bump heightmap; remap so hMin < 0 < hMax
hm_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
hm_cs = chrono.ChCoordsysd(
    chrono.ChVector3d(-7.0, 0.0, 0.0),
    chrono.QUNIT,
)
terrain.AddPatch(hm_mat, hm_cs, hm_file, 6.0, 4.0, -0.5, 0.5, True, 0.0, True)

terrain.Initialize()

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — Multi-Patch Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# Add a fixed camera for reference (positioned to see vehicle at origin)
vis.AddCamera(chrono.ChVector3d(-8.0, 0.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.5))

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording scaffolding ===
import sim_recording as rec

frame = 0  # review-only: only used in REC-gated frame capture
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

        if system.GetChTime() >= sim_end:
            break
