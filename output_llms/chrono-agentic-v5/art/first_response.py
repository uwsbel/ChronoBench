"""ARTcar wheeled vehicle on flat rigid terrain (PyChrono, Irrlicht).

Models the catalog ARTcar (a small RC-scale 4-wheeled vehicle) driving on a
flat RigidTerrain patch. System type: NSC (rigid-terrain catalog default) with
the Bullet collision system. The vehicle is initialized with an explicit world
location, orientation, contact method and mesh visualization, and is steered by
an interactive (keyboard) driver feeding steering / throttle / braking into the
vehicle subsystem stack. The real-time loop synchronizes and advances the full
driver -> terrain -> vehicle -> visualization stack, rendering at 50 frames per
second. Expected behavior: the ARTcar rests on the textured terrain and responds
to driver inputs, translating and turning across the patch.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants, then derived quantities
step_size = 1e-3                       # integration step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_fps = 50.0                      # display / capture rate (frames per second)

terrain_length = 60.0                  # rigid terrain patch size in X (m)
terrain_width = 60.0                   # rigid terrain patch size in Y (m)
terrain_height = 0.0                   # top surface of the terrain patch (m)

INIT_X = 0.0                           # vehicle spawn X (m)
INIT_Y = 0.0                           # vehicle spawn Y (m)
ARTCAR_REF_HEIGHT = 0.2                # chassis-origin height above ground at rest (m)
init_z = terrain_height + ARTCAR_REF_HEIGHT
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: facing +X

render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

# === Data paths === anchor bundled Chrono + vehicle asset trees (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === catalog ARTcar; owns its own ChSystemNSC after Initialize
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)            # rigid-terrain tire model
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.ARTcar wrapper) ===
system = vehicle.GetSystem()                             # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED for contact
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused below
# spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Footprint check: wheels must rest on (not through) the terrain after Initialize.
TIRE_RADIUS = 0.1                         # ARTcar wheel radius (m), small RC-scale
ZTOL = 0.1
veh_obj = vehicle.GetVehicle()            # cache: vehicle handle reused for spindle query
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(axle, side).z
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_height:.3f}; raise ARTCAR_REF_HEIGHT"
)

# === Terrain === flat rigid patch with a custom texture, on the vehicle's system
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: sky + camera + light + chase cam
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 3.0, 0.4)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0    # seconds 0 -> +1 steering
throttle_time = 1.0    # seconds 0 -> +1 throttle
braking_time = 0.3     # seconds 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === real-time synchronize/advance of the full subsystem stack

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === close writers, assemble review video + plot, drop frames
