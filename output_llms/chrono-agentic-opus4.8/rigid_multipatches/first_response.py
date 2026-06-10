"""HMMWV on a multi-patch rigid terrain (NSC), visualized with Irrlicht.

Models a full HMMWV high-mobility wheeled vehicle (SHAFTS engine, automatic
shafts transmission, all-wheel drive, TMEASY tires) initialized at a specified
pose with mesh visualization on every component. The terrain is a rigid
RigidTerrain made of several patches of diverse surface type:
  * two flat patches with different textures (tiled concrete / dirt),
  * a mesh-based patch carrying a raised bump (Wavefront .obj),
  * a heightmap-based patch with varying elevation (BMP heightfield).
An interactive (keyboard) driver controls steering / throttle / braking; the
real-time loop synchronizes and advances the full subsystem stack each step,
rendering the scene at a throttled frame rate. Expected behavior: the vehicle
rests on the patches under gravity and drives/steers across them when driven.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === simulation, vehicle pose, and per-patch geometry constants
time_step = 2e-3                       # integration step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_fps = 50.0                      # on-screen frame rate
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / time_step)   # precomputed once

# Vehicle initial pose (geometric-center origin for HMMWV).
INIT_X, INIT_Y = -10.0, 0.0
INIT_Z = 0.6                           # chassis origin above the flat patch top
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Flat patch sizes / placement.
FLAT_LEN, FLAT_WID = 40.0, 20.0
FLAT_A_X = -10.0                       # textured concrete patch (spawn side)
FLAT_B_X = 30.0                        # textured dirt patch (continuation)
MESH_PATCH_X = 12.0                    # bump mesh patch center
HMAP_X = 0.0                           # heightmap patch center (rolling terrain)
HMAP_LEN, HMAP_WID = 64.0, 64.0
HMAP_HMIN, HMAP_HMAX = 0.0, 1.0

# === Data paths === anchor bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper (owns its ChSystemNSC), mesh-visualized
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY: fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                       # prompt: engine type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                           # prompt: drivetrain type
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# Mesh visualization on every vehicle component (prompt requirement).
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()               # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = hmmwv.GetChassisBody()         # cache: main chassis rigid body, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain patch bodies built below
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())   # report total vehicle mass

# === Terrain === one rigid terrain holding flat / mesh-bump / heightmap patches
terrain = veh.RigidTerrain(system)

# Flat patch A — tiled concrete texture (vehicle spawn region).
mat_a = chrono.ChContactMaterialNSC()
mat_a.SetFriction(0.9)
mat_a.SetRestitution(0.01)
patch_a = terrain.AddPatch(
    mat_a,
    chrono.ChCoordsysd(chrono.ChVector3d(FLAT_A_X, 0, 0), chrono.QUNIT),
    FLAT_LEN, FLAT_WID,
)
patch_a.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)

# Flat patch B — dirt texture (continuation strip with a different surface).
mat_b = chrono.ChContactMaterialNSC()
mat_b.SetFriction(0.8)
mat_b.SetRestitution(0.01)
patch_b = terrain.AddPatch(
    mat_b,
    chrono.ChCoordsysd(chrono.ChVector3d(FLAT_B_X, 0, 0), chrono.QUNIT),
    FLAT_LEN, FLAT_WID,
)
patch_b.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)

# Mesh-based patch — a raised bump loaded from a Wavefront .obj mesh.
mat_mesh = chrono.ChContactMaterialNSC()
mat_mesh.SetFriction(0.9)
mat_mesh.SetRestitution(0.01)
patch_mesh = terrain.AddPatch(
    mat_mesh,
    chrono.ChCoordsysd(chrono.ChVector3d(MESH_PATCH_X, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
patch_mesh.SetColor(chrono.ChColor(0.5, 0.5, 0.6))

# Heightmap-based patch — varying elevation from a BMP heightfield.
mat_hmap = chrono.ChContactMaterialNSC()
mat_hmap.SetFriction(0.9)
mat_hmap.SetRestitution(0.01)
patch_hmap = terrain.AddPatch(
    mat_hmap,
    chrono.ChCoordsysd(chrono.ChVector3d(HMAP_X, 0, -2.0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    HMAP_LEN, HMAP_WID, HMAP_HMIN, HMAP_HMAX,
)
patch_hmap.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)

terrain.Initialize()

# === Footprint check === assert the wheels rest on (not through) the support
TIRE_RADIUS = 0.46                      # HMMWV tire radius (m)
ZTOL = 0.1                              # allowed wheel-bottom clearance/overlap
veh_obj = hmmwv.GetVehicle()
spindle_world = [
    veh_obj.GetSpindlePos(axle, side)
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into support: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)

# === Visualization === full Irrlicht scene: window + sky + chase camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on multi-patch rigid terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # s to reach full steering
driver.SetThrottleDelta(render_step_size / 1.0)   # s to reach full throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # s to reach full brake
driver.Initialize()

# === Main loop === real-time render-throttled Synchronize/Advance of the stack

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)                   # advances the wrapper-owned system
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
