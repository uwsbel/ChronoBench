"""HMMWV full vehicle on a rigid highway-mesh terrain patch.

Models a full HMMWV (NSC contact, TMEASY tires) driven interactively on a single
rigid-terrain patch built from the bundled Highway collision/visual meshes. The
patch contact material uses friction 0.4 / restitution 0.05, and the patch is
rotated -90 degrees about the world Z axis and placed at (6, -70, 0) so the
vehicle starts at the crossroads of the highway mesh. Expected behavior: the
HMMWV rests on the highway surface (wheels supported, no fall-through) and drives
forward under throttle, steering with the interactive driver.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants (geometry / physics) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

init_loc = chrono.ChVector3d(6, -70, 0.5)            # HMMWV spawn over the patch origin
init_rot = chrono.QuatFromAngleZ(1.57)               # face along the highway
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
contact_method = chrono.ChContactMethod_NSC          # rigid terrain -> NSC

patch_pos = chrono.ChVector3d(6, -70, 0)             # patch placed at crossroads
patch_friction = 0.4                                 # prompt: friction 0.4
patch_restitution = 0.05                             # prompt: restitution 0.05
patch_yaw = -math.pi / 2                             # prompt: -90 deg about Z

track_point = chrono.ChVector3d(-3.0, 0.0, 1.1)      # chase-camera target on chassis
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50
render_steps = math.ceil(render_step_size / step_size)   # precomputed once: render cadence

# === Vehicle (HMMWV_Full wrapper owns the ChSystem) ===
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# === System & collision (created by the veh.HMMWV_Full wrapper) ===
system = vehicle.GetSystem()                          # cache: ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

# === Terrain (single rigid patch from the Highway mesh) ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(patch_friction)
patch_mat.SetRestitution(patch_restitution)

terrain = veh.RigidTerrain(system)
quat = chrono.ChQuaterniond()
quat.SetFromAngleAxis(patch_yaw, chrono.ChVector3d(0, 0, 1))   # -90 deg yaw about Z
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(patch_pos, quat),
    chrono.GetChronoDataFile('synchrono/meshes/Highway_col.obj'),
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj"), True, True)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)
terrain.Initialize()

# === Visualization (vehicle-aware Irrlicht: window + camera + sky + light) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver (interactive, matches the real-time truth shape) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop (real-time; throttled rendering) ===

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
try:
    while vis.Run():
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

        driver_inputs = driver.GetInputs()

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
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
