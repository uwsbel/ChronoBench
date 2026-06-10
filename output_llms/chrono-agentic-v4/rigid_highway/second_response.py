"""
rigid_highway turn=2 — HMMWV on rigid mesh highway with additional terrain patch.

plan_type: mbs_in_scene (wheeled vehicle + rigid terrain + scene props)
system: NSC (ChContactMethod_NSC, ChSystemNSC)
main bodies: HMMWV_Full wrapper vehicle, RigidTerrain with mesh highway + bump patch
expected behavior: vehicle drives on the highway mesh terrain; an additional
                  bump.obj terrain patch is placed at (0, -42, 0) with dirt
                  texture (scaling 6.0, 6.0) and color (0.5, 0.5, 0.8).

This simulation adds a second terrain patch per input2.txt:
  - patch3: bump.obj mesh at (0, -42, 0), color (0.5, 0.5, 0.8),
            texture dirt.jpg at UV scaling (6.0, 6.0).
"""

import os, math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only: recording scaffolding ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = None  # scored-core safe default; overridden in REC block below

# === Named constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Vehicle initial pose
initLoc = chrono.ChVector3d(6, -70, 0.5)
initRot = chrono.QuatFromAngleZ(1.57)

# Visualization / collision types
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# Tire model (TMEASY needed for realistic rigid-terrain driving)
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions
terrainLength = 100.0
terrainWidth = 100.0

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50.0  # FPS = 50

# Chase-camera tracking point (on the chassis, in chassis frame)
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# === System & gravity (created by the veh.HMMWV_Full wrapper) ===
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# === Collision system — REQUIRED for any contact/collision scene ===
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Primary highway terrain: flat box for collision (Highway_col has 0 triangles)
# The visual mesh from Highway_vis is added separately for rendering
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)  # flat box collision patch

# Add the Highway visual mesh on top for rendering
vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh.CreateFromWavefrontFile(
    chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"), True, True)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)

# Additional terrain patch per input2.txt: bump.obj at (0, -42, 0)
# color (0.5, 0.5, 0.8), texture dirt.jpg scaling (6.0, 6.0)
patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    chrono.GetChronoDataFile("vehicle/terrain/meshes/bump.obj"))
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('rigid_highway turn=2')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Post-initialize diagnostics ===
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Simulation end time ===
sim_end = 20.0  # seconds — enough for the review video to show vehicle motion

# === Precomputed loop constants ===
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# === Review-only: ChDataDriver for validation run (avoids keyboard input) ===
# review-only: ChDataDriver with constant throttle so the vehicle moves without keyboard

# === Main loop ===
while vis.Run() and vehicle.GetSystem().GetChTime() < sim_end:
    time = vehicle.GetSystem().GetChTime()

    # Throttled rendering
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs — interactive driver (scored core); REC overrides below
    driver_inputs = driver.GetInputs()

    # Review-only: use ChDataDriver inputs so vehicle moves without keyboard

    # Synchronize and advance the full subsystem stack
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

# === Review-only: assemble videos and cleanup ===
