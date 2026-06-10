"""HMMWV climbing a deformable SCM hill with an onboard lidar and box obstacles.

Model
-----
A full HMMWV wheeled vehicle (veh.HMMWV_Full, system type ChSystemNSC owned by the
wrapper, SMC contact method) drives up a Bekker-Wong soft-soil hill built from a
heightmap (veh.SCMTerrain from terrain/height_maps/convex64.bmp). Because the soil
is deformable and the tires are TMEASY, each spindle gets an explicit collision
cylinder (family 1) so the SCM ray-cast detects sinkage and the wheels grip.

Scene elements
--------------
- HMMWV chassis + 4 spindles/wheels/tires (created by the wrapper).
- An SCM deformable hill (firm soil) the truck must climb.
- 5 box obstacles placed at deterministic, non-overlapping positions on the slope.
- An onboard lidar sensor (pychrono.sensor) riding on the chassis, with a
  point-cloud visualization filter and a save filter, driven by a ChSensorManager.

Expected behavior
-----------------
The driver applies throttle; the HMMWV accelerates from the toe of the hill and
climbs the slope, gaining elevation (chassis Z increases) while translating in +X.
The lidar samples the obstacles and terrain ahead each step. CSV logs chassis pose
and speed; a timeseries PNG is produced after the run.
"""

import os
import csv
import math
import numpy as np

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# === Named constants (geometry / physics / soil / sensing) ===
TIME_STEP = 2.0e-3                 # integration step (s)
TIRE_STEP = 1.0e-3                 # TMEASY tire substep (s); required on SCM
SIM_END = 6.0                      # simulation duration (s); ends on the hill, render < timeout
RENDER_FPS = 30.0                  # review-video frame rate

# Hill heightmap (a convex dome -> a climbable hill crest)
# A tighter footprint + taller crest makes the dome a visually obvious hill with a
# climbable grade (crest ~2.5 m over a ~15 m run from the toe).
HEIGHTMAP = "vehicle/terrain/height_maps/convex64.bmp"
SCM_SIZE_X = 32.0                  # terrain extent in X (m)
SCM_SIZE_Y = 32.0                  # terrain extent in Y (m)
SCM_H_MIN = 0.0                    # heightmap black -> this elevation (m)
SCM_H_MAX = 2.5                    # heightmap white -> crest elevation (m)
SCM_DELTA = 0.08                   # SCM grid resolution (m)

# Firm soil (Bekker-Wong) so the hill is climbable and holds ruts
SOIL_KPHI = 2.0e6                  # Bekker frictional modulus (Pa)
SOIL_KC = 0.0                      # Bekker cohesive modulus
SOIL_N = 1.1                       # Bekker exponent
SOIL_COHESION = 5.0e3             # Mohr cohesion (Pa)
SOIL_FRICTION = 30.0               # Mohr friction angle (deg)
SOIL_JANOSI = 0.01                 # Janosi shear coefficient (m)
SOIL_ELASTIC_K = 2.0e8             # elastic stiffness (Pa/m)
SOIL_DAMPING = 3.0e4               # vertical damping (Pa*s/m)

# Vehicle spawn — start near the toe of the hill, heading uphill (+X)
VEH_INIT_X = -9.0                  # spawn X (m), on the lower flank
VEH_INIT_Y = 0.0                   # spawn Y (m), centered
SUSPENSION_REF_HEIGHT = 0.55       # HMMWV chassis origin above wheel-bottom at rest (m)
ZTOL = 0.15                        # allowed wheel-bottom clearance vs terrain (m)

TIRE_FAMILY = 1                    # collision family for tire cylinders
TIRE_RAD_PAD = 0.04                # extra radius so SCM detects sinkage (m)

# Obstacles — 5 boxes laid out along the climb, deterministic & non-overlapping
NUM_OBSTACLES = 5
BOX_SIZE = 0.6                     # cube edge length (m)
BOX_MASS = 25.0                    # obstacle mass (kg)
OBSTACLE_XS = (-5.0, -2.0, 1.0, 4.0, 7.0)    # X positions along the slope (m)
OBSTACLE_YS = (2.5, -2.5, 2.5, -2.5, 0.0)    # Y offsets (m), kept off the wheel path

# Lidar parameters
LIDAR_UPDATE_RATE = 10.0           # Hz
LIDAR_W = 360                      # horizontal samples
LIDAR_H = 16                       # vertical channels
LIDAR_HFOV = 2.0 * math.pi         # full 360-degree horizontal sweep (rad)
LIDAR_MAX_VERT = 0.2618            # +15 deg (rad)
LIDAR_MIN_VERT = -0.2618           # -15 deg (rad)
LIDAR_MAX_DIST = 50.0              # max range (m)

# Derived constants (precomputed once)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # physics steps per frame
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))           # fast windowless validation
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END           # short physics check when validating

os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)       # review-video frame + motion-log dir

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
# Pre-sample the hill height at the spawn XY with a throwaway SCM terrain so the
# truck is initialized resting ON the slope (not floating or buried). The throwaway
# terrain needs its own minimal system with a collision system attached.
probe_sys = chrono.ChSystemNSC()
probe_sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # SCM requires a collision system
probe_terrain = veh.SCMTerrain(probe_sys)
probe_terrain.SetSoilParameters(
    SOIL_KPHI, SOIL_KC, SOIL_N, SOIL_COHESION,
    SOIL_FRICTION, SOIL_JANOSI, SOIL_ELASTIC_K, SOIL_DAMPING,
)
probe_terrain.Initialize(
    chrono.GetChronoDataFile(HEIGHTMAP),
    SCM_SIZE_X, SCM_SIZE_Y, SCM_H_MIN, SCM_H_MAX, SCM_DELTA,
)
spawn_ground_z = probe_terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0.0))
init_z = spawn_ground_z + SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, init_z)
init_rot = chrono.QUNIT

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # SCM deformable soil -> non-rigid tire
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# Enumerate the wrapper-created components in named locals so they are visible.
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle handle for spindle queries
# spindles/wheels/tires: veh_obj.GetAxles()[i].m_wheels[side].GetSpindle()
# joints: suspension + steering links created inside the wrapper
# terrain: the real SCMTerrain built below on this same system

# === Terrain (deformable SCM hill on the vehicle-owned system) ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    SOIL_KPHI, SOIL_KC, SOIL_N, SOIL_COHESION,
    SOIL_FRICTION, SOIL_JANOSI, SOIL_ELASTIC_K, SOIL_DAMPING,
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # colored sinkage overlay
terrain.Initialize(
    chrono.GetChronoDataFile(HEIGHTMAP),
    SCM_SIZE_X, SCM_SIZE_Y, SCM_H_MIN, SCM_H_MAX, SCM_DELTA,
)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# === Tire collision cylinders (REQUIRED for TMEASY tires on SCM) ===
# Without explicit cylinders the SCM ray-cast finds no tire geometry -> no grip.
tire_rad = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = veh_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
tire_mat.SetYoungModulus(2e7)

for axle in veh_obj.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + TIRE_RAD_PAD, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)   # never DisallowCollisionsWith(0) -> kills SCM rays
system.GetCollisionSystem().BindAll()               # rebuild all collision models after edits

# Verify the wheels rest on the hill (not buried / floating) at spawn.
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
assert wheel_bottom_z >= spawn_ground_z - ZTOL, (
    f"vehicle sinks into hill: wheel bottom z={wheel_bottom_z:.3f} vs ground "
    f"z={spawn_ground_z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{spawn_ground_z - wheel_bottom_z:.3f} m"
)

# === Obstacles (5 boxes on the slope, deterministic non-overlapping placement) ===
# Each box sits on the local terrain surface so it does not free-fall or interpenetrate.
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(2e7)

obstacles = []
for i in range(NUM_OBSTACLES):
    bx = OBSTACLE_XS[i]
    by = OBSTACLE_YS[i]
    bz = terrain.GetHeight(chrono.ChVector3d(bx, by, 0.0)) + BOX_SIZE / 2.0
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, BOX_MASS / (BOX_SIZE ** 3),
                               True, True, box_mat)
    box.SetName(f"obstacle_{i}")
    box.SetPos(chrono.ChVector3d(bx, by, bz))
    box.SetFixed(True)   # static obstacles to probe with the lidar; no spurious drift
    system.AddBody(box)
    obstacles.append(box)

# === Sensor manager + onboard lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # base fill for the lidar scene

lidar = sens.ChLidarSensor(
    chassis,                                             # ride on the chassis body
    LIDAR_UPDATE_RATE,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 1.0), chrono.QUNIT),  # mount above chassis
    LIDAR_W, LIDAR_H, LIDAR_HFOV,
    LIDAR_MAX_VERT, LIDAR_MIN_VERT, LIDAR_MAX_DIST,
)
lidar.SetName("onboard_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())                # depth-intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())             # convert to point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 540, 1.0))  # live point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_pc/"))        # save point clouds
manager.AddSensor(lidar)

# === Driver (scripted time-based control, autonomous — no human-in-the-loop) ===
class HillClimbDriver(veh.ChDriver):
    """Brief settle, then full throttle straight up the hill."""

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

driver = HillClimbDriver(veh_obj)
driver.Initialize()

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    import pychrono.irrlicht as chronoirr   # imported only for the on-screen run
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV climbing an SCM hill with lidar")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.7)
    vis.Initialize()                                                       # Initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()                                                        # outdoor sky backdrop
    vis.AddTypicalLights()                                                 # standard lighting
    vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 6.0, -12.0, init_z + 5.0),
                  chrono.ChVector3d(0.0, 0.0, SCM_H_MAX))                  # AFTER Initialize, frame the crest
    vis.AddGrid(1.0, 1.0, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, SCM_H_MIN), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))                            # ground reference grid
    vis.AttachVehicle(veh_obj)
    vis.AttachDriver(driver)

# === Main loop (Synchronize/Advance; vehicle.Advance steps the system) ===
data_writer = None
motion_writer = None
data_file = None
motion_file = None
times, pos_x, pos_z, speeds = [], [], [], []

try:
    data_file = open("simulation_data.csv", "w", newline="")        # main physics log
    motion_file = open("cam/motion_log.csv", "w", newline="")        # per-body motion contract
    data_writer = csv.writer(data_file)
    motion_writer = csv.writer(motion_file)
    data_writer.writerow(["time", "chassis_x", "chassis_y", "chassis_z", "speed", "ground_z"])
    motion_writer.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    step = 0
    frame = 0
    while (HEADLESS or vis.Run()) and system.GetChTime() < RUN_END:
        time = system.GetChTime()

        if not HEADLESS and step % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
            frame += 1

        # --- log physics this step ---
        chassis_pos = chassis.GetPos()
        chassis_vel = chassis.GetPosDt()
        speed = veh_obj.GetSpeed()
        ground_z = terrain.GetHeight(chrono.ChVector3d(chassis_pos.x, chassis_pos.y, 0.0))
        data_writer.writerow([f"{time:.5f}", f"{chassis_pos.x:.5f}", f"{chassis_pos.y:.5f}",
                              f"{chassis_pos.z:.5f}", f"{speed:.5f}", f"{ground_z:.5f}"])
        motion_writer.writerow([f"{time:.5f}", "chassis",
                                f"{chassis_pos.x:.5f}", f"{chassis_pos.y:.5f}", f"{chassis_pos.z:.5f}",
                                f"{chassis_vel.x:.5f}", f"{chassis_vel.y:.5f}", f"{chassis_vel.z:.5f}"])
        times.append(time)
        pos_x.append(chassis_pos.x)
        pos_z.append(chassis_pos.z)
        speeds.append(speed)

        # --- synchronize the full subsystem stack ---
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        if not HEADLESS:
            vis.Synchronize(time, driver_inputs)

        # --- advance every subsystem; hmmwv.Advance steps the owned system ---
        manager.Update()                 # pump the lidar each physics step
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)         # advances the wrapper-owned ChSystem
        if not HEADLESS:
            vis.Advance(TIME_STEP)
        step += 1

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:            # disk / permission on CSV writers
    import traceback
    traceback.print_exc()
    raise
finally:
    # flush partial CSV even if a step diverges
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing (timeseries plot from the logged arrays) ===
t = np.asarray(times)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax1.plot(t, np.asarray(pos_x), label="chassis X (m)")
ax1.plot(t, np.asarray(pos_z), label="chassis Z (m)")
ax1.set_ylabel("position (m)")
ax1.legend()
ax1.grid(True)
ax2.plot(t, np.asarray(speeds), color="tab:red", label="speed (m/s)")
ax2.set_xlabel("time (s)")
ax2.set_ylabel("speed (m/s)")
ax2.legend()
ax2.grid(True)
fig.suptitle("HMMWV SCM hill climb — chassis position and speed")
fig.tight_layout()
fig.savefig("simulation_timeseries.png", dpi=110)
plt.close(fig)

print(f"Done. steps logged={len(times)} "
      f"final X={pos_x[-1] if pos_x else float('nan'):.3f} "
      f"final Z={pos_z[-1] if pos_z else float('nan'):.3f}")
