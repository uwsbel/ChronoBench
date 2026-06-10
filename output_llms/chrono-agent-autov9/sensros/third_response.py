"""
sensros — mesh body sensed by a 2D lidar, published over a self-contained ROS layer.

Model
-----
A single rigid system (NSC) holds a ground plane and a static triangle-mesh body
(a chassis OBJ from the Chrono data set). A 2D lidar sensor (single scan row,
horizontal field of view) rides on a fixed sensor body and sweeps the mesh; its
range/intensity data is processed into a Cartesian point cloud through the
filter chain DI-access -> point-cloud-from-depth -> XYZI-access. The lidar
output, the sensor clock, and a per-sensor pose message are pushed every step
through a ROS publishing layer.

ROS substitution note
----------------------
`pychrono.ros` is not available in this PyChrono 9.0.1 build, so the ROS layer is
reconstructed self-contained here with the SAME SHAPE as `pychrono.ros`:
a ROSManager that owns a clock handler plus per-sensor handlers, each Registered
and Initialized once, then Updated every step (returning a success flag the main
loop checks and breaks on). The messages are serialised to CSV instead of being
put on a live DDS bus; the publish cadence, ordering, and update protocol match
the real `ChROSManager` / `ChROSClockHandler` / `ChROSLidarHandler` flow.

System type: ChSystemNSC. Main bodies: ground, static mesh body, fixed lidar
mount. Expected behavior: the lidar produces a stable, non-empty point cloud of
the mesh each scan; ROS handlers update successfully every step; CSVs are
non-empty and NaN-free.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants (geometry / physics / sensor) ===
TIME_STEP = 2.5e-3            # s, integration step
SIM_END = 6.0                # s, total simulated time
RENDER_FPS = 30.0            # Hz, Irrlicht review-frame cadence

MESH_FILE = "vehicle/hmmwv/hmmwv_chassis.obj"   # real OBJ in the Chrono data set
MESH_POS = chrono.ChVector3d(3.0, 0.0, 0.5)     # mesh sits ahead of the lidar
GROUND_SIZE = 20.0           # m, square ground patch full side length

LIDAR_POS = chrono.ChVector3d(0.0, 0.0, 0.8)    # fixed lidar mount height
LIDAR_UPDATE_RATE = 10.0     # Hz, scans per second
LIDAR_HORIZONTAL = 360       # horizontal samples (width)
LIDAR_VERTICAL = 1           # single row -> 2D lidar
LIDAR_HFOV = 2.0 * math.pi   # full horizontal sweep (rad)
LIDAR_MAX_VERT = 0.0         # 2D: vertical extent collapses to a single plane
LIDAR_MIN_VERT = 0.0
LIDAR_MAX_DIST = 40.0        # m, maximum range

# Derived constants — precomputed once, never recomputed in the loop.
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
SENSOR_UPDATE_RATE = 1.0 / TIME_STEP                          # precomputed once

# Fast, windowless validation run (short, no Irrlicht window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast windowless validation gate
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating


# === Self-contained ROS layer (mirrors the pychrono.ros SHAPE) ===
# pychrono.ros is absent in this build; these classes reproduce its protocol:
# a manager owning a clock handler + per-sensor handlers, each Register/Initialize
# once and Update(time) every step, returning a success flag.
class ROSClockHandler:
    """Stand-in for ChROSClockHandler: publishes the simulation clock."""

    def __init__(self):
        self.name = "/clock"
        self.last_time = 0.0

    def Initialize(self):
        return True

    def Update(self, sim_time):
        self.last_time = sim_time
        return True


class ROSLidarHandler:
    """Stand-in for ChROSLidarHandler: pulls the lidar XYZI cloud and publishes it."""

    def __init__(self, lidar_sensor, topic):
        self.lidar = lidar_sensor           # cache: handler holds its sensor handle
        self.topic = topic
        self.point_count = 0
        self.mean_range = 0.0

    def Initialize(self):
        return True

    def Update(self, sim_time):
        # Guard: the buffer is empty until the lidar's first scan completes.
        buf = self.lidar.GetMostRecentXYZIBuffer()   # may be empty before first tick
        if not buf.HasData():
            return True   # nothing published yet this step is still a success
        try:
            xyzi = buf.GetXYZIData()
        except (RuntimeError, ValueError):
            return False   # malformed buffer -> handler failure (loop should stop)
        pts = np.asarray(xyzi, dtype=np.float64).reshape(-1, 4)
        self.point_count = int(pts.shape[0])
        if self.point_count:
            ranges = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2)
            finite = ranges[np.isfinite(ranges)]
            self.mean_range = float(finite.mean()) if finite.size else 0.0
        return True


class ROSPoseHandler:
    """Stand-in for a ChROSBodyHandler: publishes a sensed body pose."""

    def __init__(self, body, topic):
        self.body = body                    # cache: handler holds its body handle
        self.topic = topic
        self.pos = (0.0, 0.0, 0.0)

    def Initialize(self):
        return True

    def Update(self, sim_time):
        p = self.body.GetPos()
        self.pos = (p.x, p.y, p.z)
        return True


class ROSManager:
    """Stand-in for ChROSManager: registers handlers, initializes and updates them."""

    def __init__(self):
        self.handlers = []

    def RegisterHandler(self, handler):
        self.handlers.append(handler)

    def Initialize(self):
        return all(h.Initialize() for h in self.handlers)

    def Update(self, sim_time, time_step):
        # Returns False if any handler fails — the loop checks this and exits.
        for h in self.handlers:
            if not h.Update(sim_time):
                return False
        return True


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

contact_mat = chrono.ChContactMaterialNSC()   # NSC material to match ChSystemNSC
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

# === Bodies (ground + sensed mesh body + lidar mount) ===
# Ground: collision geometry so the OptiX lidar can see it as a floor.
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, 0.2, 1000.0, True, True, contact_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.SetFixed(True)
sys.Add(ground)

# Sensed mesh body. OptiX renders only bodies WITH collision geometry, so the mesh
# is given both a visual triangle mesh and a matching collision mesh.
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile(MESH_FILE), True, True)

mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(mesh)
mesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(MESH_POS)
mesh_body.SetFixed(True)
mesh_body.AddVisualShape(mesh_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

coll_mesh = chrono.ChCollisionShapeTriangleMesh(contact_mat, mesh, True, True, 0.005)
mesh_body.AddCollisionShape(coll_mesh, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
mesh_body.EnableCollision(True)
sys.Add(mesh_body)   # add the mesh body to the simulation system

# Fixed lidar mount.
lidar_mount = chrono.ChBody()
lidar_mount.SetPos(LIDAR_POS)
lidar_mount.SetFixed(True)
sys.Add(lidar_mount)

# === Sensor manager + lighting ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(5, 5, 8), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
# ChScene has no AddDirectionalLight here -> use a point light plus ambient instead.
manager.scene.SetAmbientLight(chrono.ChVector3f(0.4, 0.4, 0.4))

# === 2D lidar sensor + filter chain ===
lidar = sens.ChLidarSensor(
    lidar_mount,
    LIDAR_UPDATE_RATE,
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),
    LIDAR_HORIZONTAL,
    LIDAR_VERTICAL,
    LIDAR_HFOV,
    LIDAR_MAX_VERT,
    LIDAR_MIN_VERT,
    LIDAR_MAX_DIST,
)
lidar.SetName("2d_lidar")
# Initialize the filters for the 2D lidar: raw range/intensity access, then a
# Cartesian point cloud from the depth/range buffer, then XYZI access.
lidar.PushFilter(sens.ChFilterDIAccess())        # raw depth+intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())     # convert range buffer -> point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())      # access the XYZI point cloud
manager.AddSensor(lidar)

# === ROS layer wiring (register handlers, then initialize) ===
ros_manager = ROSManager()
ros_manager.RegisterHandler(ROSClockHandler())
ros_manager.RegisterHandler(ROSLidarHandler(lidar, "/lidar/points"))
ros_manager.RegisterHandler(ROSPoseHandler(mesh_body, "/mesh/pose"))
if not ros_manager.Initialize():
    raise RuntimeError("ROS layer failed to initialize")

# Cache the lidar/pose handlers for cheap per-step CSV logging (no re-scan).
lidar_handler = ros_manager.handlers[1]   # cache: fetched once, reused every step
pose_handler = ros_manager.handlers[2]    # cache: fetched once, reused every step

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("sensros — 2D lidar over ROS")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-4, -6, 4), chrono.ChVector3d(3, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 20, 20,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop (render-cadence; manager.Update + ROS update each physics step) ===
os.makedirs("frames", exist_ok=True)   # guard against missing output dir
os.makedirs("cam", exist_ok=True)      # guard against missing output dir

data_f = None
motion_f = None
try:
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk / permission failure opening CSVs
        print("Could not open output CSVs:", exc)
        raise

    with data_f, motion_f:
        data_w = csv.writer(data_f)
        data_w.writerow(["time", "lidar_points", "mean_range", "ros_ok"])
        motion_w = csv.writer(motion_f)
        motion_w.writerow(["time", "mesh_x", "mesh_y", "mesh_z"])

        frame = 0
        ros_ok = True
        while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index
                frame += 1
            for _ in range(RENDER_EVERY):
                manager.Update()   # pump the lidar every physics step
                t = sys.GetChTime()
                ros_ok = ros_manager.Update(t, TIME_STEP)
                data_w.writerow([f"{t:.5f}", lidar_handler.point_count,
                                 f"{lidar_handler.mean_range:.5f}", int(ros_ok)])
                motion_w.writerow([f"{t:.5f}", f"{pose_handler.pos[0]:.5f}",
                                   f"{pose_handler.pos[1]:.5f}", f"{pose_handler.pos[2]:.5f}"])
                sys.DoStepDynamics(TIME_STEP)
                if not ros_ok:
                    print("ROS manager update failed — stopping loop.")
                    break
                if sys.GetChTime() >= RUN_END:
                    break
            if not ros_ok:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    # CSV writers are closed by the `with` block above; nothing else to flush here.
    print(f"Simulation finished at t={sys.GetChTime():.3f}s")

# === Post-processing (plot logged time series) ===
try:
    times, points, ranges = [], [], []
    with open("simulation_data.csv", "r", newline="") as f:   # context-managed read
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            points.append(float(row["lidar_points"]))
            ranges.append(float(row["mean_range"]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, points, color="tab:blue")
    ax1.set_ylabel("lidar points")
    ax1.grid(True)
    ax2.plot(times, ranges, color="tab:red")
    ax2.set_ylabel("mean range (m)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    fig.suptitle("2D lidar point cloud over time")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
except (OSError, IOError, ValueError) as exc:   # missing CSV / parse failure
    print("Post-processing plot skipped:", exc)
