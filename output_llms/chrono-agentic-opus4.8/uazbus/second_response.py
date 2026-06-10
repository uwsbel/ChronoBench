"""UAZBUS wheeled-vehicle double-lane-change simulation (PyChrono 9.0.0, Irrlicht).

Models the catalog UAZBUS wheeled vehicle (NSC contact) spawned at world
x = -40 m on a flat rigid terrain patch textured with concrete. A scripted
ChDataDriver executes an ISO-style double lane change: the vehicle accelerates
straight, steers left then right (lane change out and back), repeats the
maneuver, and finally brakes to a stop. Expected behavior: the bus drives
forward from the spawn point, weaves through the two lane-change segments
without rolling over, and decelerates to rest at the end of the run.

System type: NSC (rigid terrain catalog vehicle). Main bodies: UAZBUS chassis +
four wheels/spindles on a single rigid RigidTerrain patch.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / timing constants (no bare literals downstream)
step_size = 2e-3                       # integration step (s)
sim_end = 16.0                         # total simulated time (s)
render_fps = 50.0                      # review-render cadence
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)   # precomputed once

INIT_X = -40.0                         # spawn x (prompt: moved from 0 to -40)
INIT_Y = 0.0
INIT_Z = 0.5                           # chassis-origin height above flat ground
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

TERRAIN_LENGTH = 200.0                 # X size of the rigid patch (covers the run)
TERRAIN_WIDTH = 100.0                  # Y size of the rigid patch
TERRAIN_TOP_Z = 0.0                    # flat ground top plane


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === UAZBUS catalog wrapper (owns its ChSystemNSC)
uazbus = veh.UAZBUS()
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)
uazbus.SetChassisFixed(False)                         # MANDATORY: fixed chassis won't move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
uazbus.SetTireType(veh.TireModelType_TMEASY)          # rigid-terrain tire model
uazbus.SetTireStepSize(step_size)
uazbus.Initialize()

uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
system = uazbus.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = uazbus.GetChassisBody()                     # cache: main chassis rigid body, reused every step
# wheels/spindles: uazbus.GetVehicle().GetSpindlePos(axle, side); joints: suspension + steering inside wrapper
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())

# Footprint check: wheel bottoms must rest on (not through) the flat ground.
TIRE_RADIUS = 0.45                     # UAZBUS tire radius (approx, for the rest check)
ZTOL = 0.10
veh_obj = uazbus.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(a, s)
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into ground: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs ground top z={TERRAIN_TOP_Z:.3f}; raise INIT_Z"
)

# === Terrain === flat rigid patch with concrete texture (prompt: concrete.jpg)
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
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Driver === scripted double-lane-change + braking schedule (ChDataDriver)
# Entries: (time, steering, throttle, braking). The vehicle accelerates straight,
# performs two lane-change weaves (left then right, repeated), then brakes.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),     # start at rest
    veh.DataDriverEntry(1.0, 0.0, 0.6, 0.0),     # accelerate straight
    veh.DataDriverEntry(4.0, 0.0, 0.6, 0.0),     # reach cruising speed
    veh.DataDriverEntry(5.0, 0.40, 0.5, 0.0),    # lane change 1: steer left
    veh.DataDriverEntry(6.0, -0.40, 0.5, 0.0),   # return to lane: steer right
    veh.DataDriverEntry(7.0, 0.0, 0.5, 0.0),     # straighten
    veh.DataDriverEntry(9.0, -0.40, 0.5, 0.0),   # lane change 2: steer right
    veh.DataDriverEntry(10.0, 0.40, 0.5, 0.0),   # return to lane: steer left
    veh.DataDriverEntry(11.0, 0.0, 0.5, 0.0),    # straighten
    veh.DataDriverEntry(12.0, 0.0, 0.0, 0.8),    # brake toward a stop
    veh.DataDriverEntry(16.0, 0.0, 0.0, 0.8),    # hold brake
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Visualization === full Irrlicht vehicle scene (window + sky + camera + lights)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()              # vehicle truths use a directional light
vis.AttachVehicle(veh_obj)

# === Main loop === real-time Synchronize/Advance over the full subsystem stack
os.makedirs("cam", exist_ok=True)      # guard against missing output dir
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
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
        veh_obj.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        uazbus.Advance(step_size)      # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
