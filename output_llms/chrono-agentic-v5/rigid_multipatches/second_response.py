"""HMMWV full-vehicle driving on a single mesh-based rigid highway terrain.

Models a four-wheeled HMMWV (NSC contact, rigid terrain) spawned on a single
rigid-terrain patch built from a collision mesh (`Highway_col.obj`) and given a
separate high-resolution visual mesh (`Highway_vis.obj`) attached to the terrain
ground body via a ChVisualShapeTriangleMesh. The vehicle is placed at
(6, -70, 0.5) and is steered by an interactive driver bound to the Irrlicht
visual system. Expected behavior: the HMMWV rests on the highway mesh surface
(wheels supported, no fall-through) and drives forward under throttle.

System type: NSC (rigid terrain catalog vehicle).
Main bodies: HMMWV chassis + 4 wheels/spindles, rigid-terrain ground body.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
time_step = 2e-3                       # integration step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_fps = 50.0                      # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

INIT_LOC = chrono.ChVector3d(6, -70, 0.5)   # vehicle spawn (prompt-specified)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

TERRAIN_FRICTION = 0.9                 # single-patch contact friction
TERRAIN_RESTITUTION = 0.01            # single-patch contact restitution
CONTACT_THICKNESS = 0.01              # patch collision mesh thickness (sweep radius)

# Highway terrain mesh assets (collision mesh + separate visual mesh).
COL_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")
VIS_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === HMMWV full model on rigid terrain (NSC)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # rigid-terrain compatible tire
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                      # cache: main chassis rigid body, reused
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())  # report total vehicle mass

# === Terrain === single mesh patch (collision mesh) + separate visual mesh
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

# Single patch built from the collision mesh; thickness -> sweep-sphere radius.
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    COL_MESH,
    True,                # connected mesh
    CONTACT_THICKNESS,   # contact thickness (sweep sphere radius)
)

# Attach a high-resolution visual mesh to the terrain ground body.
vis_trimesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(VIS_MESH, True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_trimesh)
vis_shape.SetName("highway_vis")
vis_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())

terrain.Initialize()

# === Footprint assert === wheels must rest on (not through) the mesh surface
veh_obj = hmmwv.GetVehicle()
TIRE_RADIUS = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: tire radius
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
support_z = terrain.GetHeight(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0))  # mesh height at spawn
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= support_z - 0.30, (
    f"vehicle wheels start below highway surface: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z={support_z:.3f}; raise INIT_LOC.z"
)

# === Visualization === vehicle-aware Irrlicht window: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway mesh terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                              # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance over driver/terrain/vehicle/vis
realtime_timer = chrono.ChRealtimeStepTimer()


frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()


            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)
            vis.Advance(time_step)

            realtime_timer.Spin(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
