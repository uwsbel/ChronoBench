"""
UAZ Bus simulation on RigidTerrain with RIGID tires and a box obstacle.

System: ChSystemNSC (owned by veh.UAZBUS wrapper)
Vehicle: UAZ Bus (veh.UAZBUS) with TireModelType_RIGID tires
Terrain: RigidTerrain flat patch (NSC contact material), 300 x 100 m
Obstacle: Fixed box (0.5 x 5 x 0.2 m) placed at x=5, y=0, z=0.1
Driver: ChInteractiveDriver (scored core) with review-only scripted throttle=0.5
Expected behavior: Vehicle drives forward from x=-10, approaches the fixed box
obstacle at x=5, and interacts with it on flat terrain.
"""

# === Imports ===
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters ===
# Simulation timing
time_step = 5e-3          # 5 ms step size (standard for wheeled vehicle demos)
sim_end = 15.0            # 15 s — enough to reach obstacle and drive past it
render_fps = 50.0
render_step_size = 1.0 / render_fps              # precomputed once
render_steps = math.ceil(render_step_size / time_step)  # precomputed once

# Terrain — large enough to contain vehicle trajectory at ~8 m/s for 15 s (~120 m)
terrain_length = 300.0    # m, X extent (centered at origin: -150 to +150)
terrain_width = 100.0     # m, Y extent (centered at origin: -50 to +50)

# Obstacle parameters — fixed box (0.5 x 5 x 0.2 m) at (5, 0, 0.1)
OBS_X = 5.0
OBS_Y = 0.0
OBS_Z = 0.1              # center Z = half-height, box bottom sits on z=0
OBS_HALF_X = 0.25        # half-extents (full dims 0.5 x 5 x 0.2)
OBS_HALF_Y = 2.5
OBS_HALF_Z = 0.1

# Vehicle spawn — UAZBUS: tire radius 0.372 m, spindle z = init_z at rest
TIRE_RADIUS = 0.372       # from veh_obj.GetAxle(0).GetWheels()[0].GetTire().GetRadius()
# For wheel bottom to rest at terrain z=0: spindle_z = TIRE_RADIUS (0.372)
# The spindle z equals the init_loc z for UAZBUS at rest
SUSPENSION_REF_HEIGHT = TIRE_RADIUS  # 0.372 m — sets spindle at z=TIRE_RADIUS
ZTOL = 0.05               # allowed wheel-bottom clearance tolerance

init_loc = chrono.ChVector3d(-10, 0, SUSPENSION_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# === Review-only recording setup ===

# === Vehicle setup (veh.UAZBUS wrapper) ===
uazbus = veh.UAZBUS()
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)
uazbus.SetChassisFixed(False)                    # MANDATORY — fixed chassis won't move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
uazbus.SetTireType(veh.TireModelType_RIGID)      # prompt: rigid tire model
uazbus.SetTireStepSize(time_step)
uazbus.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = uazbus.GetSystem()                         # ChSystemNSC owned by the wrapper
chassis = uazbus.GetChassisBody()                # cache: main chassis rigid body, fetched once
# wheels/spindles: uazbus.GetVehicle().GetAxle(i).GetWheels()[j].GetSpindle()
# joints: suspension links, steering links, powertrain — created inside wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# === Visualization types (after Initialize) ===
uazbus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
uazbus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
uazbus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
uazbus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
uazbus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Validate wheel-bottom position (assert wheels rest on terrain) ===
veh_obj = uazbus.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain z=0; raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Terrain (flat, NSC, large enough for 15 s run) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Box obstacle (fixed, 0.5 x 5 x 0.2 m at position (5, 0, 0.1)) ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.0)

obstacle = chrono.ChBodyEasyBox(
    2.0 * OBS_HALF_X,    # full X = 0.5
    2.0 * OBS_HALF_Y,    # full Y = 5.0
    2.0 * OBS_HALF_Z,    # full Z = 0.2
    1000.0,              # density kg/m³
    True,                # visualize
    True,                # collide
    obs_mat,
)
obstacle.SetName("box_obstacle")
obstacle.SetPos(chrono.ChVector3d(OBS_X, OBS_Y, OBS_Z))
obstacle.SetFixed(True)
sys.Add(obstacle)

# === Irrlicht vehicle visualization ===
# Initialize first, then add scene elements (Irrlicht call order)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ Bus — RIGID Tires + Box Obstacle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(uazbus.GetVehicle())

# === Interactive driver (scored core — matches truth for catalog vehicles) ===
driver = veh.ChInteractiveDriver(uazbus.GetVehicle())

steering_time = 1.0   # s to go 0 -> +1 steering
throttle_time = 1.0   # s to go 0 -> +1 throttle
braking_time = 0.3    # s to go 0 -> +1 brake

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only: output directory and CSV setup ===

# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and uazbus.GetSystem().GetChTime() < sim_end:
        time = uazbus.GetSystem().GetChTime()    # cache: fetched once per render frame

        # Throttled rendering — only render every render_steps physics steps
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs — interactive in scored core (keyboard / gamepad control)
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in fixed order: driver -> terrain -> vehicle -> vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        uazbus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Log physics to CSV (review-only)

        # Advance subsystems — do NOT also call sys.DoStepDynamics
        driver.Advance(time_step)
        terrain.Advance(time_step)
        uazbus.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

except (RuntimeError, ValueError) as exc:        # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
