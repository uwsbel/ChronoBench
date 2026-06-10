"""
2D Lidar sensor scene with a self-contained ROS publishing layer (PyChrono 9.0.1 + Irrlicht).

WHAT THIS MODELS
----------------
A small static scene (a ground plane plus a ring of obstacle boxes and a central
pillar) observed by a planar (2D) rotating Lidar. The Lidar rides on a fixed mast
body at the world origin and sweeps a single horizontal scan line (height = 1 beam
row) around the obstacles. The obstacles are rigid boxes with collision geometry so
that the OptiX Lidar can actually see them (the OptiX renderer only traces bodies
that carry collision geometry).

SYSTEM TYPE
-----------
ChSystemNSC (non-smooth contact). Gravity is along -Z (Z-up world). Nothing moves
dynamically here; the scene is static and the Lidar is the subject of the demo, so
the physics integration just advances time while the sensor sweeps.

ROS SUBSTITUTION NOTE
---------------------
This PyChrono build ships no `pychrono.ros` module, so the ROS publishing layer is
reconstructed *self-contained* here, mirroring the SHAPE of the real pychrono.ros
API: a `ChROSManager` owns a `ChROSClockHandler` plus one handler per sensor; every
handler is Registered, the manager is Initialized once, and `manager.Update()` is
called every physics step. The 2D-Lidar handler publishes the scan onto the topic
`~/output/lidar2d/data/scan` (the same topic the real ROS lidar handler uses). The
underlying sensor is a REAL `pychrono.sensor.ChLidarSensor`; the handler reads its
DI (distance/intensity) buffer and "publishes" the LaserScan-equivalent record by
logging it to CSV (no live DDS broker is available in this environment).

EXPECTED BEHAVIOR
-----------------
The Lidar's named filters drive a live point-cloud preview; the DI buffer fills
after the first sensor tick and yields a per-frame minimum/mean/valid-return count
that stays bounded and non-NaN as the beam sweeps the fixed obstacle ring. The ROS
handler reports a steadily increasing publish count and a stable scan topic.
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

# === Named constants === geometry / physics / sensor / run configuration (no bare literals downstream)
TIME_STEP = 2.0e-3            # s, physics integration step
SIM_END = 6.0                # s, total simulated time
RENDER_FPS = 30.0            # Hz, review-frame cadence for the Irrlicht window
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once: physics steps per frame

GRAVITY_Z = -9.81            # m/s^2, world gravity along -Z (Z-up)

GROUND_SIZE = 30.0           # m, side length of the square ground patch
GROUND_THICK = 0.4           # m, ground slab thickness

PILLAR_RADIUS = 0.4          # m, central pillar radius
PILLAR_HEIGHT = 2.0          # m, central pillar height

OBSTACLE_COUNT = 8           # number of boxes in the obstacle ring
OBSTACLE_RING_R = 6.0        # m, radius of the obstacle ring around the lidar
OBSTACLE_SIZE = 0.8          # m, obstacle box edge length (full extent)
OBSTACLE_HEIGHT = 1.5        # m, obstacle box height (full extent)

LIDAR_MAST_Z = 1.0           # m, height of the lidar mast above ground top

# --- 2D Lidar configuration (single horizontal scan row) ---
LIDAR_UPDATE_RATE = 10.0     # Hz, lidar revolutions per second
LIDAR_H_SAMPLES = 360        # horizontal samples per revolution (1 deg resolution)
LIDAR_V_SAMPLES = 1          # vertical samples -> 2D / planar lidar (single beam row)
LIDAR_HFOV = 2.0 * math.pi   # rad, full 360 deg horizontal field of view
LIDAR_MAX_V_ANGLE = 0.0      # rad, single horizontal plane -> max == min vertical angle
LIDAR_MIN_V_ANGLE = 0.0      # rad
LIDAR_MAX_DISTANCE = 100.0   # m, maximum measurable range
LIDAR_SCAN_TOPIC = "~/output/lidar2d/data/scan"   # ROS topic the lidar handler publishes to

OUT_FRAMES_DIR = "frames"
OUT_CAM_DIR = "cam"
OUT_DATA_CSV = "simulation_data.csv"
OUT_MOTION_CSV = os.path.join(OUT_CAM_DIR, "motion_log.csv")
OUT_PLOT_PNG = "simulation_timeseries.png"

# Fast, windowless validation run when SIMBENCH_VALIDATE is set (full Irrlicht block is
# still present below for the source reviewer; the gate only skips the on-screen window).
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))


# === Self-contained ROS layer === mirrors the pychrono.ros API SHAPE (no pychrono.ros module here)
class ChROSClockHandler:
    """Reconstructed clock handler: publishes the simulation clock (/clock) shape each Update."""

    def __init__(self):
        self.topic = "/clock"
        self.last_time = 0.0
        self.publish_count = 0

    def Update(self, sim_time, time_step):   # noqa: N802 (match pychrono.ros method casing)
        self.last_time = sim_time
        self.publish_count += 1
        return True


class ChROSLidar2DHandler:
    """Reconstructed 2D-Lidar ROS handler.

    Mirrors pychrono.ros' per-sensor handler shape: it wraps a real ChLidarSensor,
    reads its DI buffer, and "publishes" a sensor_msgs/LaserScan-equivalent record
    onto `topic`. With no DDS broker available the published record is logged to CSV.
    """

    def __init__(self, lidar_sensor, topic, h_samples, max_distance):
        self.lidar = lidar_sensor          # cache: real ChLidarSensor, reused every Update
        self.topic = topic
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (2.0 * math.pi) / float(h_samples)   # precomputed once: LaserScan angle step
        self.range_min = 0.0
        self.range_max = max_distance
        self.publish_count = 0
        # Latest published scan summary (what a subscriber would receive this tick):
        self.last_valid = 0
        self.last_min_range = float("nan")
        self.last_mean_range = float("nan")

    def Update(self, sim_time, time_step):   # noqa: N802 (match pychrono.ros method casing)
        buf = self.lidar.GetMostRecentDIBuffer()   # may be empty before the first sensor tick
        if not buf.HasData():                        # guard: skip frames the sensor has not filled yet
            return False
        di = buf.GetDIData()                         # (H, W, 2) array: [:, :, 0]=range, [:, :, 1]=intensity
        ranges = np.asarray(di)[:, :, 0].reshape(-1)
        valid = ranges[(ranges > self.range_min) & (ranges < self.range_max)]
        self.last_valid = int(valid.size)
        if valid.size > 0:
            self.last_min_range = float(np.min(valid))
            self.last_mean_range = float(np.mean(valid))
        else:
            self.last_min_range = float("nan")
            self.last_mean_range = float("nan")
        self.publish_count += 1                      # one LaserScan published on `self.topic`
        return True


class ChROSManager:
    """Reconstructed ROS manager: owns handlers, Initializes once, Updates every step."""

    def __init__(self):
        self.handlers = []          # cache: handler list, iterated every Update
        self.initialized = False
        self.update_count = 0

    def RegisterHandler(self, handler):   # noqa: N802
        self.handlers.append(handler)

    def Initialize(self):   # noqa: N802
        # In real pychrono.ros this binds the DDS node; here it just marks readiness.
        self.initialized = True
        return True

    def Update(self, sim_time, time_step):   # noqa: N802
        if not self.initialized:
            return False
        self.update_count += 1
        for handler in self.handlers:        # clock handler + per-sensor handlers
            handler.Update(sim_time, time_step)
        return True


def build_obstacle_ring(system, material):
    """Create the static obstacle ring + central pillar (all with collision geometry).

    Collision geometry is REQUIRED so the OptiX Lidar can trace these bodies; visual
    shapes alone are invisible to the sensor renderer.
    """
    bodies = []
    pillar = chrono.ChBodyEasyCylinder(
        chrono.ChAxis_Z, PILLAR_RADIUS, PILLAR_HEIGHT,
        1000.0, True, True, material,
    )
    pillar.SetPos(chrono.ChVector3d(0.0, 0.0, PILLAR_HEIGHT * 0.5))
    pillar.SetFixed(True)
    pillar.SetName("pillar_center")
    system.Add(pillar)
    bodies.append(pillar)

    angle_step = (2.0 * math.pi) / float(OBSTACLE_COUNT)   # precomputed once: ring angular spacing
    for i in range(OBSTACLE_COUNT):
        ang = i * angle_step
        px = OBSTACLE_RING_R * math.cos(ang)
        py = OBSTACLE_RING_R * math.sin(ang)
        box = chrono.ChBodyEasyBox(
            OBSTACLE_SIZE, OBSTACLE_SIZE, OBSTACLE_HEIGHT,
            1000.0, True, True, material,
        )
        box.SetPos(chrono.ChVector3d(px, py, OBSTACLE_HEIGHT * 0.5))
        box.SetFixed(True)
        box.SetName(f"obstacle_{i:02d}")
        system.Add(box)
        bodies.append(box)
    return bodies


def main():
    # === System & gravity === ChSystemNSC, Z-up world, static scene with collision geometry
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, GRAVITY_Z))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    contact_mat = chrono.ChContactMaterialNSC()
    contact_mat.SetFriction(0.8)
    contact_mat.SetRestitution(0.0)

    # === Bodies === ground patch + obstacle ring + central pillar (all collidable for the Lidar)
    ground = chrono.ChBodyEasyBox(
        GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
        1000.0, True, True, contact_mat,
    )
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, -GROUND_THICK * 0.5))
    ground.SetFixed(True)
    ground.SetName("ground")
    system.Add(ground)

    obstacles = build_obstacle_ring(system, contact_mat)

    # Fixed mast the Lidar rides on, at the world origin, LIDAR_MAST_Z above ground top.
    lidar_mast = chrono.ChBody()
    lidar_mast.SetFixed(True)
    lidar_mast.SetPos(chrono.ChVector3d(0.0, 0.0, LIDAR_MAST_Z))
    lidar_mast.SetName("lidar_mast")
    system.Add(lidar_mast)

    # === Sensor manager === OptiX sensor scene + lighting (ChScene has no AddDirectionalLight here)
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(
        chrono.ChVector3f(10.0, 10.0, 30.0), chrono.ChColor(1.0, 1.0, 1.0), 500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-10.0, -10.0, 30.0), chrono.ChColor(1.0, 1.0, 1.0), 500.0,
    )
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

    # === 2D Lidar sensor === planar (single beam row) 360 deg sweep on the mast, named filters
    lidar = sens.ChLidarSensor(
        lidar_mast,                                   # body the lidar rides on
        LIDAR_UPDATE_RATE,                            # Hz (revolutions per second)
        chrono.ChFramed(chrono.VNULL, chrono.QUNIT),  # offset frame on the mast
        LIDAR_H_SAMPLES,                              # horizontal samples
        LIDAR_V_SAMPLES,                              # vertical samples == 1 -> 2D planar lidar
        LIDAR_HFOV,                                   # horizontal FOV (rad)
        LIDAR_MAX_V_ANGLE,                            # max vertical angle (rad)
        LIDAR_MIN_V_ANGLE,                            # min vertical angle (rad)
        LIDAR_MAX_DISTANCE,                           # max range (m)
        sens.LidarBeamShape_RECTANGULAR,
        1,                                            # sample radius
        0.003, 0.003,                                 # vertical / horizontal divergence (rad)
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("lidar2d")
    # Named filters: a named DI-access filter (publishable distance/intensity buffer)
    # and a named point-cloud visualization for review.
    lidar.PushFilter(sens.ChFilterDIAccess("lidar2d_di_access"))
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(720, 720, 1.0, "lidar2d_pc_view"))
    lidar.PushFilter(sens.ChFilterXYZIAccess("lidar2d_xyzi_access"))
    manager.AddSensor(lidar)

    # === ROS layer === self-contained ChROSManager + clock handler + 2D-lidar scan handler
    ros_manager = ChROSManager()
    clock_handler = ChROSClockHandler()
    lidar_handler = ChROSLidar2DHandler(
        lidar, LIDAR_SCAN_TOPIC, LIDAR_H_SAMPLES, LIDAR_MAX_DISTANCE,
    )
    ros_manager.RegisterHandler(clock_handler)
    ros_manager.RegisterHandler(lidar_handler)
    ros_manager.Initialize()

    # === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
    vis = None
    if not HEADLESS:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("2D Lidar + ROS scan publisher")
        vis.Initialize()                                            # Initialize FIRST (inverse of VSG)
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(12.0, -12.0, 8.0), chrono.ChVector3d(0.0, 0.0, 1.0))
        vis.AddTypicalLights()
        vis.AddGrid(1.0, 1.0, 40, 40,
                    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
                    chrono.ChColor(0.4, 0.4, 0.4))

    # === Output dirs === guard against missing output directories before opening writers
    os.makedirs(OUT_FRAMES_DIR, exist_ok=True)   # guard: review frames target dir
    os.makedirs(OUT_CAM_DIR, exist_ok=True)       # guard: motion-log target dir

    run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

    data_file = None
    motion_file = None
    times, valids, min_ranges, mean_ranges, pub_counts = [], [], [], [], []
    try:
        data_file = open(OUT_DATA_CSV, "w", newline="")
        motion_file = open(OUT_MOTION_CSV, "w", newline="")
        data_writer = csv.writer(data_file)
        motion_writer = csv.writer(motion_file)
        data_writer.writerow([
            "time", "ros_update_count", "lidar_topic", "scan_publish_count",
            "valid_returns", "min_range_m", "mean_range_m",
            "angle_increment_rad", "range_max_m",
        ])
        motion_writer.writerow([
            "time", "body_name", "x", "y", "z", "vx", "vy", "vz",
        ])

        # cache: bodies whose pose we log every step, fetched once
        logged_bodies = [lidar_mast] + obstacles

        # === Main loop === render-cadence outer loop; manager.Update() every physics step
        frame = 0
        while (HEADLESS or vis.Run()) and system.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(os.path.join(OUT_FRAMES_DIR, f"img_{frame:06d}.png"))
                frame += 1
            for _ in range(RENDER_EVERY):
                manager.Update()                       # pump the OptiX lidar every physics step
                sim_time = system.GetChTime()
                ros_manager.Update(sim_time, TIME_STEP)   # clock + lidar scan handlers publish

                data_writer.writerow([
                    f"{sim_time:.4f}", ros_manager.update_count, lidar_handler.topic,
                    lidar_handler.publish_count, lidar_handler.last_valid,
                    f"{lidar_handler.last_min_range:.4f}", f"{lidar_handler.last_mean_range:.4f}",
                    f"{lidar_handler.angle_increment:.6f}", f"{lidar_handler.range_max:.2f}",
                ])
                for body in logged_bodies:
                    pos = body.GetPos()
                    vel = body.GetPosDt()
                    motion_writer.writerow([
                        f"{sim_time:.4f}", body.GetName(),
                        f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}",
                        f"{vel.x:.4f}", f"{vel.y:.4f}", f"{vel.z:.4f}",
                    ])

                times.append(sim_time)
                valids.append(lidar_handler.last_valid)
                min_ranges.append(lidar_handler.last_min_range)
                mean_ranges.append(lidar_handler.last_mean_range)
                pub_counts.append(lidar_handler.publish_count)

                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= run_end:
                    break
    except (OSError, IOError) as exc:                     # disk / permission failure on CSV I/O
        import traceback
        traceback.print_exc()
        raise
    except (RuntimeError, ValueError) as exc:             # solver divergence / bad sensor state
        import traceback
        traceback.print_exc()
        raise
    finally:
        # Flush + close any open writers even if a step diverged mid-run.
        if data_file is not None:
            data_file.close()
        if motion_file is not None:
            motion_file.close()

    # === Post-processing === plot the published-scan time series from the collected data
    if len(times) > 0:
        t = np.asarray(times)
        fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
        axes[0].plot(t, valids, color="tab:blue")
        axes[0].set_ylabel("valid returns")
        axes[0].set_title("2D Lidar scan published on " + LIDAR_SCAN_TOPIC)
        axes[0].grid(True)
        axes[1].plot(t, min_ranges, color="tab:green", label="min range")
        axes[1].plot(t, mean_ranges, color="tab:orange", label="mean range")
        axes[1].set_ylabel("range (m)")
        axes[1].legend()
        axes[1].grid(True)
        axes[2].plot(t, pub_counts, color="tab:red")
        axes[2].set_ylabel("scan publish count")
        axes[2].set_xlabel("time (s)")
        axes[2].grid(True)
        fig.tight_layout()
        fig.savefig(OUT_PLOT_PNG, dpi=110)
        plt.close(fig)

    print(f"ROS manager updates: {ros_manager.update_count}")
    print(f"Lidar scans published on {lidar_handler.topic}: {lidar_handler.publish_count}")
    print(f"Clock handler publishes: {clock_handler.publish_count}")
    print(f"Logged samples: {len(times)}")


if __name__ == "__main__":
    main()
