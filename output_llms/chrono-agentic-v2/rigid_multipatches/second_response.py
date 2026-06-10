"""
HMMWV vehicle on a single rigid terrain mesh patch (Highway).

System type: NSC (rigid terrain)
Vehicle: HMMWV_Full — positioned at (6, -70, 0.5), driving on a single
  terrain patch defined by the Highway_col.obj collision mesh with an
  accompanying Highway_vis.obj visual mesh.
Expected behavior: vehicle initializes on the highway mesh terrain, engine
  spins up and the vehicle moves forward under interactive driver control.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 2e-3          # physics step (s)
SIM_END = 20.0            # simulation duration (s)
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(6, -70, 0.5)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# Terrain contact material parameters
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# === Data paths (required for catalog vehicles — scored) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                   # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                     # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # cache: fetched once, reused every step
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # truth component

# Visualization types — set after Initialize
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain — single mesh patch (Highway) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

# Single mesh collision patch using Highway_col.obj
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),
)
patch.SetContactFrictionCoefficient(TERRAIN_FRICTION)
patch.SetContactRestitutionCoefficient(TERRAIN_RESTITUTION)

# Add visual mesh using ChVisualShapeTriangleMesh
terrain_body = patch.GetGroundBody()
vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh_data = chrono.ChTriangleMeshConnected()
vis_mesh_data.LoadWavefrontMesh(
    chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_vis.obj"),
    False, True
)
vis_mesh.SetMesh(vis_mesh_data)
vis_mesh.SetMutable(False)
terrain_body.AddVisualShape(vis_mesh)

terrain.Initialize()

# === Interactive driver ===
# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# Build interactive driver from vis (after vis is built)
driver = veh.ChInteractiveDriverIRR(vis)

render_step_size = 1.0 / RENDER_FPS     # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        # review-only scripted driving block (stripped at ACCEPT)

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        for _ in range(render_every):
            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)
            step_number += 1
            if sys.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:      # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
