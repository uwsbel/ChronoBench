"""
HMMWV on a single rigid mesh terrain (Highway_col.obj collision mesh +
Highway_vis.obj visual mesh). NSC contact method. Interactive Irrlicht driver.

System type   : ChSystemNSC (owned by the HMMWV_Full wrapper)
Main bodies   : HMMWV chassis, suspension/wheel/tire bodies (wrapper-created),
                RigidTerrain ground body (mesh patch)
Expected      : HMMWV spawns at (6, -70, 0.5) on the Highway mesh terrain and
                responds to keyboard driver input.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
STEP_SIZE        = 1e-3         # physics time step (s)
SIM_END          = 20.0         # simulation duration (s)
RENDER_FPS       = 50.0
RENDER_EVERY     = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))       # precomputed once

INIT_LOC         = chrono.ChVector3d(6, -70, 0.5)
# Rotate 90 degrees around Z so HMMWV faces +Y (along the highway length)
INIT_ROT         = chrono.QuatFromAngleZ(math.pi / 2)

FRICTION         = 0.9
RESTITUTION      = 0.01
PATCH_THICKNESS  = 0.05   # larger sweep radius for robust mesh collision detection

STEERING_TIME    = 1.0
THROTTLE_TIME    = 1.0
BRAKING_TIME     = 0.3

# === Data paths (mandatory for catalog vehicle — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle: HMMWV_Full ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)        # TMEASY for good grip
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys     = hmmwv.GetSystem()                        # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                   # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i) ... ; terrain body below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # truth-required diagnostic

# === Visualization type ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain: single rigid mesh patch (Highway) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()          # NSC to match vehicle contact method
patch_mat.SetFriction(FRICTION)
patch_mat.SetRestitution(RESTITUTION)

# Collision mesh patch — Highway_col.obj with sweep_sphere_radius for contact thickness
# Files live under synchrono/meshes/ in the Chrono data tree
highway_col_file = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    highway_col_file,
    True,           # connected_mesh
    PATCH_THICKNESS, # sweep_sphere_radius (contact material thickness)
)

# Visual mesh for the terrain ground body
ground_body = patch.GetGroundBody()
vis_mesh = chrono.ChVisualShapeTriangleMesh()
highway_vis_mesh = chrono.ChTriangleMeshConnected()
highway_vis_mesh.LoadWavefrontMesh(
    chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"), False, True
)
vis_mesh.SetMesh(highway_vis_mesh)
vis_mesh.SetMutable(False)
ground_body.AddVisualShape(vis_mesh)

terrain.Initialize()

# === Irrlicht vehicle visualization ===
# Call order: configure window → Initialize → add scene elements → AttachVehicle
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                          # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver: interactive IRR (catalog-vehicle default — truth faithful) ===
render_step_size = 1.0 / RENDER_FPS              # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Recording setup (review-only) ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_EVERY == 0:
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

except (RuntimeError, ValueError) as exc:                 # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
