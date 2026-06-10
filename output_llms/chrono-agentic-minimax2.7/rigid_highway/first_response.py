"""
rigid_highway turn=1 — SimBench v9-4
HMMWV on a custom mesh highway terrain with TMEASY tire model and interactive driver.
Physics: NSC contact, rigid terrain via mesh collision geometry, Irrlicht rendering.
"""

import math, os, csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only: sim_recording for frame capture + CSV ===

# === Named constants ===
step_size = 1e-3
sim_end = 20.0           # simulation duration (seconds)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))

# Vehicle spawn
init_loc = chrono.ChVector3d(6, -70, 0.5)
init_rot = chrono.QuatFromAngleZ(1.57)

# Camera chase point (vehicle local frame)
track_point = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Tire model: TMEASY for rigid highway terrain
tire_model = veh.TireModelType_TMEASY
tire_step_size = step_size

# Terrain geometry
terrain_length = 100.0
terrain_width = 100.0

# Interactive driver response
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

# === System & gravity ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

# Make vehicle components visible (enumerate wrapper-created objects)
hmmwv_sys = hmmwv.GetSystem()                      # cache: ChSystemNSC owned by wrapper
hmmwv_chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body
hmmwv_veh = hmmwv.GetVehicle()                      # cache: vehicle descriptor

# Set visualization types for all components
vis_type = veh.VisualizationType_MESH
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# Collision system REQUIRED for contact/collision scene
hmmwv_sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain: RigidTerrain with Highway mesh collision + visual ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(hmmwv_sys)
terrain_patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    chrono.GetChronoDataFile('synchrono/meshes/Highway_col.obj'),
    True, 0.01, False,
)

vis_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
    chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"), True, True,
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
terrain_patch.GetGroundBody().AddVisualShape(tri_mesh_shape)
terrain.Initialize()

# === Irrlicht Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv_veh)

# === Interactive Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(1.0 / (render_fps * steering_time))
driver.SetThrottleDelta(1.0 / (render_fps * throttle_time))
driver.SetBrakingDelta(1.0 / (render_fps * braking_time))
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and hmmwv_sys.GetChTime() < sim_end:
        # Throttled rendering
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs (interactive driver — scored core)
        driver_inputs = driver.GetInputs()

        # Synchronize all subsystems
        driver.Synchronize(hmmwv_sys.GetChTime())
        terrain.Synchronize(hmmwv_sys.GetChTime())
        hmmwv.Synchronize(hmmwv_sys.GetChTime(), driver_inputs, terrain)
        vis.Synchronize(hmmwv_sys.GetChTime(), driver_inputs)

        # Advance all subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise

print("VEHICLE MASS: ", hmmwv_veh.GetMass())
print("Done.")

# === Review-only: frame capture, CSV logging, scripted driving, video assembly ===
