"""HMMWV climbing a deformable SCM soil hill with an onboard lidar.

Model: a full-model HMMWV (TMEASY tires) drives up a Bekker-Wong SCM soft-soil
heightmap hill while an onboard rotating lidar sensor scans the surroundings.
Five box obstacles are scattered (rejection-sampled, non-overlapping) on the
terrain so the lidar has structure to return ranges from.

System type: NSC owned by the veh.HMMWV_Full wrapper (SMC contact method on the
wrapper). Bullet collision system. Main bodies: HMMWV chassis + 4 spindles/tires,
the SCM deformable terrain, and 5 fixed box obstacles.

Expected behavior: the vehicle accelerates from rest, drives forward, and climbs
the crest of the hill (chassis Z rises by ~2 m), leaving ruts in the soil; the
lidar produces a depth/intensity point stream throughout the run.
"""

import math
import os
import numpy as np

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Named constants: timing, geometry, soil, sensor ===
time_step = 2e-3                       # integration step (s)
tire_step = 1e-3                       # TMEASY tire substep (s)
sim_end = 12.0                         # simulation duration (s)
render_fps = 25.0                      # review-video frame rate

TERRAIN_LEN = 60.0                     # SCM patch X size (m)
TERRAIN_WID = 30.0                     # SCM patch Y size (m)
SCM_RES = 0.08                         # SCM grid resolution (m)
HILL_HEIGHT = 2.6                      # crest height of the heightmap hill (m)

SUSPENSION_REF_HEIGHT = 0.5            # HMMWV chassis-origin height above wheel-bottom at rest
VEH_INIT_X = -22.0                     # spawn near the flat foot of the hill (m)
VEH_INIT_Y = 0.0

N_BOXES = 5                            # number of scattered obstacles
BOX_SIZE = 1.0                         # cube edge length (m)
BOX_CLEAR = 4.0                        # min center spacing between boxes (m)
VEH_KEEPOUT = 6.0                      # keep boxes clear of the spawn point (m)

LIDAR_HZ = 10.0                        # lidar update rate (Hz)
LIDAR_W = 800                          # horizontal samples
LIDAR_H = 40                           # vertical channels
LIDAR_HFOV = 2.0 * math.pi             # full 360 deg horizontal scan (rad)
LIDAR_MAX_V = 0.26                     # +15 deg upper vertical angle (rad)
LIDAR_MIN_V = -0.26                    # -15 deg lower vertical angle (rad)
LIDAR_RANGE = 60.0                     # max range (m)

# === Heightmap hill: synthesize a 16-bit PNG ridge once ===
# WHAT: build a smooth ridge that rises toward +X so the vehicle ascends a crest.
# WHY: SCMTerrain's heightmap Initialize maps white->max_z, black->min_z.
HEIGHTMAP_PATH = os.path.abspath("hill_heightmap.png")
hm_n = 256                             # heightmap pixel resolution
_xs = np.linspace(-1.0, 1.0, hm_n)     # precomputed once: normalized X ramp
_ramp = 0.5 * (1.0 + np.tanh(2.5 * _xs))               # 0..1 smooth ramp along X
_ridge = np.tile(_ramp[np.newaxis, :], (hm_n, 1))      # invariant across Y
_img16 = (_ridge * 65535.0).astype(np.uint16)


def _write_png16(path, arr):
    """Write a 16-bit grayscale PNG without external imaging deps."""
    import struct
    import zlib

    height, width = arr.shape

    def _chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = bytearray()
    for row in arr:
        raw.append(0)                                   # filter type 0 per scanline
        raw.extend(row.byteswap().tobytes() if arr.dtype.byteorder == "<" else row.tobytes())
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 16, 0, 0, 0, 0)
    with open(path, "wb") as fh:                         # context manager: always flush/close
        fh.write(sig)
        fh.write(_chunk(b"IHDR", ihdr))
        fh.write(_chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        fh.write(_chunk(b"IEND", b""))


_write_png16(HEIGHTMAP_PATH, np.ascontiguousarray(_img16.astype(">u2")))


def hill_height_at(x):
    """Sample the synthesized ridge height (m) at world X (Y-invariant)."""
    u = (x + TERRAIN_LEN / 2.0) / TERRAIN_LEN            # 0..1 across patch
    u = min(max(u, 0.0), 1.0)
    ramp = 0.5 * (1.0 + math.tanh(2.5 * (2.0 * u - 1.0)))
    return HILL_HEIGHT * ramp


# === Vehicle: HMMWV_Full on SCM (TMEASY tires required) ===
# WHAT: build + initialize the wrapper, which owns its ChSystem.
# WHY: the wrapper creates the system, chassis, spindles, suspension + steering joints.
init_z = hill_height_at(VEH_INIT_X) + SUSPENSION_REF_HEIGHT     # rest the wheels on the soil
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, init_z)
init_rot = chrono.QuatFromAngleZ(0.0)                          # facing +X (uphill)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)        # SCM needs a slip/grip tire, not RIGID
hmmwv.SetTireStepSize(tire_step)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                          # ChSystem owned by the wrapper
chassis = hmmwv.GetChassisBody()                    # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()                        # cache: vehicle subsystem handle, reused every step
# spindles/tires: veh_obj.GetAxles()[i].m_wheels[j].GetSpindle(); joints live inside the wrapper.

# Set the Bullet collision system on the wrapper-owned system after Initialize.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain: SCM Bekker-Wong soft soil over the heightmap hill ===
# WHAT: deformable soil so the tires sink and leave ruts while climbing.
# WHY: SCMTerrain needs the collision system to exist first (set just above).
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,     # Bekker_Kphi  — firm frictional modulus (Pa)
    0,       # Bekker_Kc    — cohesive modulus
    1.1,     # Bekker_n     — exponent
    0,       # Mohr_cohesion
    30,      # Mohr_friction (deg)
    0.01,    # Janosi_shear (m)
    2e8,     # elastic_K (Pa/m)
    3e4,     # damping_R (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)     # colored sinkage overlay
terrain.Initialize(HEIGHTMAP_PATH, TERRAIN_LEN, TERRAIN_WID, 0.0, HILL_HEIGHT, SCM_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 40, 40)

# === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
# WHAT: explicit per-spindle cylinders so SCM ray-casts detect tire contact.
# WHY: TMEASY tires add no collision geometry; without these no ruts/grip form.
TIRE_FAMILY = 1
tire_rad = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()   # precomputed once
tire_w = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()      # precomputed once
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)

for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)       # tires never self-collide
system.GetCollisionSystem().BindAll()                   # rebuild models so ray-casts see cylinders

# Assert the wheels rest on (not through) the soil at spawn.
wheel_bottom_z = min(
    veh_obj.GetSpindlePos(a, s).z
    for a in range(veh_obj.GetNumberAxles())
    for s in (veh.LEFT, veh.RIGHT)
) - tire_rad
ground_z = hill_height_at(VEH_INIT_X)
assert wheel_bottom_z >= ground_z - 0.15, (
    f"vehicle sinks into hill: wheel bottom z={wheel_bottom_z:.3f} vs soil z={ground_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {ground_z - wheel_bottom_z:.3f} m"
)

# === Box obstacles: rejection-sampled, non-overlapping, clear of spawn ===
# WHAT: 5 fixed cubes scattered on the terrain for the lidar to detect.
# WHY: rejection sampling keeps them from overlapping each other or the vehicle.
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(2e7)

rng = np.random.default_rng(7)                      # fixed seed: reproducible layout
box_centers = []
attempts = 0
while len(box_centers) < N_BOXES and attempts < 2000:
    attempts += 1
    bx = float(rng.uniform(-TERRAIN_LEN / 2 + 4.0, TERRAIN_LEN / 2 - 4.0))
    by = float(rng.uniform(-TERRAIN_WID / 2 + 4.0, TERRAIN_WID / 2 - 4.0))
    if math.hypot(bx - VEH_INIT_X, by - VEH_INIT_Y) < VEH_KEEPOUT:
        continue                                    # reject: too close to the vehicle spawn
    if any(math.hypot(bx - cx, by - cy) < BOX_CLEAR for cx, cy in box_centers):
        continue                                    # reject: overlaps an already-placed box
    box_centers.append((bx, by))

assert len(box_centers) == N_BOXES, f"only placed {len(box_centers)}/{N_BOXES} boxes"

for i, (bx, by) in enumerate(box_centers):
    bz = hill_height_at(bx) + BOX_SIZE / 2.0         # sit the cube on the soil surface
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, 800, True, True, box_mat)
    box.SetName(f"obstacle_{i}")
    box.SetPos(chrono.ChVector3d(bx, by, bz))
    box.SetFixed(True)
    system.AddBody(box)
system.GetCollisionSystem().BindAll()                # bind the new obstacle collision models

# === Driver: scripted time-based throttle/steer (no human-in-the-loop) ===
class HillDriver(veh.ChDriver):
    """Brief brake settle, then steady throttle straight uphill."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.85)
            self.SetBraking(0.0)
        self.SetSteering(0.0)


driver = HillDriver(veh_obj)
driver.Initialize()

# === Sensors: onboard rotating lidar on the chassis ===
# WHAT: a 360 deg lidar riding ~1.8 m above the chassis origin.
# WHY: access + save filters only — NO live point-cloud window (would block/timeout).
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1.0, 1.0, 1.0), 1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-20, 20, 30), chrono.ChColor(0.6, 0.6, 0.6), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.8), chrono.QUNIT)
lidar = sens.ChLidarSensor(
    chassis,                       # ride on the chassis -> onboard / chase-style scan
    LIDAR_HZ,
    lidar_offset,
    LIDAR_W, LIDAR_H,
    LIDAR_HFOV,
    LIDAR_MAX_V, LIDAR_MIN_V,
    LIDAR_RANGE,
)
lidar.SetName("onboard_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())                  # depth/intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())               # convert ranges -> XYZI point cloud
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/sensor_lidar/"))   # save point clouds -> video later
lidar.PushFilter(sens.ChFilterXYZIAccess())                # XYZI buffer access
manager.AddSensor(lidar)

# === Visualization: vehicle-aware Irrlicht window + chase camera ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV climbing an SCM soil hill with onboard lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop: render-cadence outer loop; Synchronize/Advance per step ===
render_every = max(1, round(1.0 / (render_fps * time_step)))         # precomputed once


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
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            manager.Update()                        # pump the lidar every physics step
            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)                # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
