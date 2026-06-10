"""MAN 5t military truck driving across a grass-textured rigid terrain with an
onboard lidar sensor and a field of randomly scattered obstacle boxes.

System type: NSC (wrapper-owned ChSystem from veh.MAN_5t, SMC contact method).
Main bodies:
  - MAN_5t wheeled truck (chassis + 3 axles, TMEASY tires) driven by a scripted
    open-loop driver (brief brake, then steady throttle with a gentle steer sweep).
  - Flat RigidTerrain patch textured with grass.
  - A set of rigid obstacle boxes, placed by rejection sampling so none overlap
    each other or the truck's spawn corridor.
Sensing:
  - A roof-mounted lidar (ChLidarSensor) returns depth/intensity (ChFilterDIAccess),
    a point cloud from depth (ChFilterPCfromDepth), and XYZI points
    (ChFilterXYZIAccess), read each frame from the XYZI buffer.
Expected behavior: the truck launches forward, sweeps slightly in steering, and
drives through the box field while the lidar reports a non-empty point cloud of
the surrounding obstacles and ground.
"""

import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np


# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2e-3                       # integration step (s)
tire_step = 1e-3                       # TMEASY tire substep (s)
sim_end = 12.0                         # simulated duration (s)
render_fps = 30.0                      # review-video frame rate

terrain_length = 120.0                 # X extent of the ground patch (m)
terrain_width = 120.0                  # Y extent of the ground patch (m)
terrain_height = 0.0                   # top surface Z of the flat patch (m)

veh_init_x = -40.0                     # truck spawn X (m), drives toward +X
veh_init_y = 0.0                       # truck spawn Y (m)
SUSPENSION_REF_HEIGHT = 0.55           # MAN 5t chassis origin above wheel-bottom (m)
veh_init_z = terrain_height + SUSPENSION_REF_HEIGHT
TIRE_RADIUS = 0.60                     # approximate MAN 5t tire radius (m)
ZTOL = 0.15                            # allowed wheel-bottom clearance vs support (m)

num_boxes = 25                         # number of random obstacle boxes
box_size = 1.2                         # cube edge length (m)
box_density = 600.0                    # obstacle box density (kg/m^3)
box_field_x = (-25.0, 45.0)            # box-field X range (m)
box_field_y = (-22.0, 22.0)            # box-field Y range (m)
box_min_gap = 3.0                      # min center-to-center spacing between boxes (m)
corridor_half_w = 4.0                  # keep boxes clear of the start corridor (m)

rng_seed = 12345                       # deterministic obstacle layout

# === Derived constants === computed once before the loop (precomputed once)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # physics steps per frame
lidar_update_rate = 10.0               # lidar revolutions per second (Hz)
lidar_hfov = 2.0 * math.pi             # full 360-degree horizontal sweep (rad)
lidar_max_v = math.radians(15.0)       # upper vertical beam angle (rad)
lidar_min_v = math.radians(-15.0)      # lower vertical beam angle (rad)
lidar_w = 360                          # horizontal samples per revolution
lidar_h = 32                           # vertical channels
lidar_max_dist = 100.0                 # max return distance (m)

# === System & vehicle (MAN_5t wrapper owns the ChSystem) ===
# The wrapper creates the system, chassis rigid body, 3 axles/spindles, the
# suspension + steering joints, the engine/transmission, and the TMEASY tires.
truck = veh.MAN_5t()
truck.SetContactMethod(chrono.ChContactMethod_SMC)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)
truck.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(veh_init_x, veh_init_y, veh_init_z), chrono.QUNIT)
)
truck.SetTireType(veh.TireModelType_TMEASY)     # slip/grip tire so the truck actually drives
truck.SetTireStepSize(tire_step)
truck.Initialize()

truck.SetChassisVisualizationType(chrono.VisualizationType_MESH)
truck.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(chrono.VisualizationType_MESH)
truck.SetTireVisualizationType(chrono.VisualizationType_MESH)

system = truck.GetSystem()                     # cache: ChSystem owned by the wrapper, reused below
chassis = truck.GetChassisBody()               # cache: main chassis rigid body, reused every step

# === Collision system === Bullet narrowphase for vehicle/terrain/box contacts
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Footprint check: wheel bottoms must rest on (not through) the terrain top.
veh_obj = truck.GetVehicle()                   # cache: ChWheeledVehicle handle
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"truck sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs "
    f"terrain top z={terrain_height:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{terrain_height - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid grass-textured patch under the truck
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.6, 0.35))
terrain.Initialize()

# === Obstacle boxes === rejection-sampled so none overlap each other or corridor
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(1e7)

rng = np.random.default_rng(rng_seed)          # deterministic obstacle layout
placed = []                                    # accepted (x, y) box centers
attempts = 0
max_attempts = num_boxes * 400
while len(placed) < num_boxes and attempts < max_attempts:
    attempts += 1
    bx = float(rng.uniform(*box_field_x))
    by = float(rng.uniform(*box_field_y))
    # reject boxes sitting in the truck's straight-ahead start corridor
    if bx < veh_init_x + 8.0 and abs(by - veh_init_y) < corridor_half_w:
        continue
    # reject boxes too close to an already-placed box (no overlap)
    if any((bx - px) ** 2 + (by - py) ** 2 < box_min_gap ** 2 for px, py in placed):
        continue
    placed.append((bx, by))

for i, (bx, by) in enumerate(placed):
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, box_density, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, terrain_height + box_size / 2.0))
    box.SetFixed(True)                         # static obstacles the lidar should see
    box.SetName(f"obstacle_box_{i}")
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.75, 0.35, 0.15))
    system.AddBody(box)

# === Driver === scripted open-loop control (brief brake, then drive + steer sweep)
class ScriptedDriver(veh.ChDriver):
    """Time-based control law: settle on the brake, then throttle up with a gentle
    sinusoidal steering sweep so the truck weaves through the obstacle field."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.6)
            self.SetBraking(0.0)
        self.SetSteering(0.25 * math.sin(0.4 * time))

driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Sensors === onboard 360-degree lidar on the truck roof + scene lighting
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(60, 60, 100), chrono.ChColor(1.0, 1.0, 1.0), 1000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 2.4), chrono.QUNIT)   # roof mount
lidar = sens.ChLidarSensor(
    chassis,                  # rides on the chassis (onboard, moves with the truck)
    lidar_update_rate,
    lidar_offset,
    lidar_w,
    lidar_h,
    lidar_hfov,
    lidar_max_v,
    lidar_min_v,
    lidar_max_dist,
    sens.LidarBeamShape_RECTANGULAR,
    1,                        # sample radius
    0.003,                    # vertical divergence (rad)
    0.003,                    # horizontal divergence (rad)
    sens.LidarReturnMode_STRONGEST_RETURN,
    0.05,                     # clip near (m)
)
lidar.SetName("onboard_lidar")
# Depth/intensity -> point cloud -> XYZI access; read points via the XYZI buffer.
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t with onboard lidar in a box field")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 2.0), 12.0, 1.0)   # chase point, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(veh_init_x - 10.0, -10.0, 6.0),
              chrono.ChVector3d(veh_init_x, 0.0, 1.0))
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Output setup === guard against a missing output directory before logging


# === Main loop === render-cadence outer loop; physics + sensors in the inner batch
last_lidar_points = 0                          # most recent lidar XYZI point count
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            truck.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            manager.Update()                   # pump the lidar every physics step

            xyzi_buf = lidar.GetMostRecentXYZIBuffer()   # may be empty before first lidar tick
            if xyzi_buf.HasData():             # guard: only read a filled buffer
                last_lidar_points = int(xyzi_buf.Width * xyzi_buf.Height)


            driver.Advance(time_step)
            terrain.Advance(time_step)
            truck.Advance(time_step)           # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video(s) + plot, then clean frame dirs
