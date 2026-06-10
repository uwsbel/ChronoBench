"""Gator wheeled-vehicle demo on flat rigid terrain (NSC contact).

Models a John Deere Gator catalog vehicle driving on a flat RigidTerrain patch
and rendered with the vehicle-aware Irrlicht visual system. System type: NSC
(rigid-terrain catalog vehicle). Main bodies: the Gator chassis + four wheels/
tires (created by the veh.Gator wrapper) and a single rigid terrain patch.

Three configuration choices define this scene:
  * The vehicle is drawn with PRIMITIVE visual shapes (not mesh assets), giving a
    simplified box/cylinder rendering of chassis, suspension, wheels and tires.
  * The chassis carries a simple PRIMITIVE collision box welded to the chassis
    body (the wrapper's own chassis collision stays NONE), so the body collides
    with the world using a cheap box rather than a triangle mesh.
  * The interactive driver is made deliberately sluggish: large rise-times for
    steering / throttle / braking mean the per-step input deltas are small, so
    keyboard commands ramp in slowly and the controls feel low-response.

Expected behavior: the Gator rests on the terrain and, when driven, accelerates
and steers gradually because of the slow driver response.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / physics / driver-response constants
time_step = 1e-3                       # integration step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_fps = 50.0                      # review render cadence (frames/s)

INIT_X, INIT_Y = 0.0, 0.0             # spawn XY on the terrain patch
SUSPENSION_REF_HEIGHT = 0.4           # chassis origin above wheel-bottom at rest
TERRAIN_TOP_Z = 0.0                   # flat patch top surface height
TERRAIN_LENGTH = 100.0                # patch size in X (m)
TERRAIN_WIDTH = 100.0                 # patch size in Y (m)
TIRE_RADIUS = 0.285                   # Gator tire radius (m), for footprint assert
ZTOL = 0.05                           # allowed wheel-bottom clearance vs support

# Chassis primitive collision box (full extents, m) and its local offset.
CHASSIS_BOX = (2.4, 1.2, 0.7)
CHASSIS_BOX_OFFSET_Z = 0.4

# Driver response: LARGE rise-times -> SMALL deltas -> sluggish, low-response
# controls (the slower the rise-time, the longer keyboard inputs take to apply).
STEERING_RISE_TIME = 4.0              # seconds 0 -> +1 steering (slow)
THROTTLE_RISE_TIME = 4.0              # seconds 0 -> +1 throttle (slow)
BRAKING_RISE_TIME = 2.0              # seconds 0 -> +1 braking (slow)

init_z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT          # precomputed once
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

render_step_size = 1.0 / render_fps                       # precomputed once
render_steps = math.ceil(render_step_size / time_step)   # steps per rendered frame


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === John Deere Gator on rigid terrain (NSC); wrapper owns system
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)     # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)  # add a primitive box below instead
vehicle.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)            # rigid-terrain tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# Simplified PRIMITIVE rendering of every vehicle subsystem (no mesh assets).
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.Gator wrapper) ===
system = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis_body = vehicle.GetChassisBody()       # cache: main chassis rigid body, reused below
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); terrain patch below.
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Chassis collision === simple PRIMITIVE box welded to the chassis body
chassis_mat = chrono.ChContactMaterialNSC()   # match the NSC system
chassis_mat.SetFriction(0.7)
chassis_mat.SetRestitution(0.0)
chassis_body.AddCollisionShape(
    chrono.ChCollisionShapeBox(chassis_mat, CHASSIS_BOX[0], CHASSIS_BOX[1], CHASSIS_BOX[2]),
    chrono.ChFramed(chrono.ChVector3d(0, 0, CHASSIS_BOX_OFFSET_Z), chrono.QUNIT),
)
chassis_body.EnableCollision(True)
system.GetCollisionSystem().BindAll()         # bind the post-initialize collision edit

# === Footprint check === confirm wheels rest on (not through) the terrain
veh_obj = vehicle.GetVehicle()                # cache: fetched once for spindle query
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch under the vehicle
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + sky + chase camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.0), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                      # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive driver with deliberately slow (low-response) deltas
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_RISE_TIME)   # small delta -> slow steering
driver.SetThrottleDelta(render_step_size / THROTTLE_RISE_TIME)   # small delta -> slow throttle
driver.SetBrakingDelta(render_step_size / BRAKING_RISE_TIME)     # small delta -> slow braking
driver.Initialize()


# === Main loop === real-time interactive drive: synchronize + advance full stack
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:       # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)            # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:         # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
