"""SimBench lidar scene — dual lidar sensing of a fixed box target.

Model
-----
A non-smooth-contact (NSC) rigid-body scene whose subject is a single fixed
cube (``chrono.ChBodyEasyBox``) standing on a large ground plate. Two OptiX
lidar sensors (``sens.ChLidarSensor``) observe the cube:

* ``lidar_3d`` — a multi-channel 3D scanning lidar attached to the cube. It
  sweeps a horizontal field of view across several vertical channels and
  produces a depth/intensity (DI) image plus an XYZI point cloud.
* ``lidar_2d`` — a planar 2D lidar with a single vertical channel
  (``height = 1``), modelling a horizontal line scanner.

OptiX only returns hits for bodies that carry COLLISION geometry, so every
visible body in this scene is built with collision shapes enabled. The
expected behavior: both lidars return finite ranges off the cube and the
ground; the logged point count and min/mean range stay stable across the run
(the scene is static, so the geometry does not move).

System type: ``ChSystemNSC`` (rigid contact, Bullet collision detection).
Visualization: Irrlicht review window; lidar point clouds saved via
``ChFilterSavePtCloud``.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless-safe backend for the post-run plot
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / sensor parameters (no bare literals downstream)
TIME_STEP = 4.0e-3          # s, raised for OptiX render headroom under the 300 s budget
SIM_END = 4.0               # s, total simulated time
RENDER_FPS = 25.0           # Hz, Irrlicht review-frame cadence
GROUND_SIZE = 20.0          # m, side length of the square ground plate
GROUND_THICK = 0.4          # m, ground plate thickness
SIDE = 2.0                  # m, cube edge length (the box target dimension)
BOX_DENSITY = 1000.0        # kg/m^3, cube material density

LIDAR_UPDATE_RATE = 10.0    # Hz, lidar scan rate
LIDAR_MAX_DIST = 100.0      # m, maximum return distance
LIDAR_3D_W = 480            # horizontal samples (3D lidar)
LIDAR_3D_H = 32             # vertical channels (3D lidar)
LIDAR_2D_W = 480            # horizontal samples (2D lidar)
LIDAR_2D_H = 1              # one vertical channel -> planar 2D scan
LIDAR_HFOV = 2.0 * math.pi  # rad, full 360-degree horizontal sweep
LIDAR_3D_VMAX = 0.2618      # rad, +15 deg upper vertical extent
LIDAR_3D_VMIN = -0.2618     # rad, -15 deg lower vertical extent
LIDAR_2D_VANGLE = 0.0       # rad, single horizontal plane for the 2D lidar
LIDAR_MOUNT_BACK = 6.0      # m, lidar stand-off distance behind the cube (-X)
LIDAR_MOUNT_Z = 1.0         # m, lidar mounting height above ground

# Derived placement (precomputed once)
GROUND_Z = -GROUND_THICK / 2.0          # ground top surface sits at z = 0
BOX_Z = SIDE / 2.0                       # cube rests on the ground (z = 0 plane)
LIDAR_POS = chrono.ChVector3d(-LIDAR_MOUNT_BACK, 0.0, LIDAR_MOUNT_Z)

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run


def _range_stats(di_array):
    """Return (point_count, min_range, mean_range) for finite, non-zero hits.

    OptiX writes 0.0 range for beams that miss; values >= LIDAR_MAX_DIST are
    no-returns. Both are excluded so the statistics describe real surface hits.
    """
    ranges = di_array[:, :, 0].reshape(-1)
    valid = ranges[(ranges > 0.0) & (ranges < LIDAR_MAX_DIST)]
    if valid.size == 0:
        return 0, float("nan"), float("nan")
    return int(valid.size), float(valid.min()), float(valid.mean())


# === System & gravity === NSC rigid-body system with Bullet collision for OptiX returns
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === ground plate + cube target (both carry collision geometry for lidar hits)
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                              BOX_DENSITY, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
sys.Add(ground)

box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.0)

# The box object replaces any mesh target: a simple cube with collision geometry.
box = chrono.ChBodyEasyBox(SIDE, SIDE, SIDE, BOX_DENSITY, True, True, box_mat)
box.SetPos(chrono.ChVector3d(0, 0, BOX_Z))
box.SetFixed(True)
box.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.5, 0.9))
sys.Add(box)

# === Sensors === sensor manager + scene lighting (ChScene has no AddDirectionalLight in 9.0.1)
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 20), chrono.ChColor(1.0, 1.0, 1.0), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-10, -10, 20), chrono.ChColor(0.8, 0.8, 0.8), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# 3D scanning lidar attached to the box target, looking back toward the lidar stand-off.
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-LIDAR_MOUNT_BACK, 0.0, LIDAR_MOUNT_Z - BOX_Z),
    chrono.QUNIT,
)
lidar_3d = sens.ChLidarSensor(
    box,                      # parent body the lidar rides on (the cube target)
    LIDAR_UPDATE_RATE,
    lidar_offset,
    LIDAR_3D_W, LIDAR_3D_H,
    float(LIDAR_HFOV),
    float(LIDAR_3D_VMAX), float(LIDAR_3D_VMIN),
    float(LIDAR_MAX_DIST),
)
lidar_3d.SetName("lidar_3d")
lidar_3d.PushFilter(sens.ChFilterDIAccess())           # access depth/intensity image
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())        # convert depth image -> point cloud
lidar_3d.PushFilter(sens.ChFilterSavePtCloud("lidar_3d_pc/"))  # save point cloud frames
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())         # access XYZI point cloud buffer
manager.AddSensor(lidar_3d)

# Additional 2D lidar: one vertical channel -> a single horizontal scan plane.
lidar_2d = sens.ChLidarSensor(
    box,
    LIDAR_UPDATE_RATE,
    lidar_offset,
    LIDAR_2D_W, LIDAR_2D_H,
    float(LIDAR_HFOV),
    float(LIDAR_2D_VANGLE), float(LIDAR_2D_VANGLE),
    float(LIDAR_MAX_DIST),
)
lidar_2d.SetName("lidar_2d")
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterSavePtCloud("lidar_2d_pc/"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Dual Lidar Box Scene")
    vis.Initialize()                                   # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-9, -7, 5), chrono.ChVector3d(0, 0, 1))  # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(1.0, 1.0, 20, 20,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid

# === Derived loop constants === precomputed once (never recomputed in the hot loop)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # physics steps per frame
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END          # short physics check when validating

os.makedirs("frames", exist_ok=True)   # guard against missing review-frame output dir
os.makedirs("cam", exist_ok=True)       # guard against missing motion-log dir

# === Main loop === render-cadence outer loop; pump sensors + log CSV every physics step
data_file = None
motion_file = None
try:
    try:
        data_file = open("simulation_data.csv", "w", newline="")
        motion_file = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:   # disk full / permission denied on output open
        import traceback
        traceback.print_exc()
        raise

    data_writer = csv.writer(data_file)
    data_writer.writerow([
        "time",
        "lidar3d_points", "lidar3d_min_range", "lidar3d_mean_range",
        "lidar2d_points", "lidar2d_min_range", "lidar2d_mean_range",
    ])
    motion_writer = csv.writer(motion_file)
    motion_writer.writerow(["time", "box_x", "box_y", "box_z", "box_vx", "box_vy", "box_vz"])

    box_body = box                       # cache: subject body fetched once, reused every step

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1

        for _ in range(render_every):
            manager.Update()             # pump both lidars EVERY physics step
            t = sys.GetChTime()

            # Lidar statistics (guard: buffers are empty before the first sensor tick).
            p3, min3, mean3 = 0, float("nan"), float("nan")
            buf3 = lidar_3d.GetMostRecentDIBuffer()
            if buf3.HasData():           # guard: skip frames the 3D lidar has not filled yet
                p3, min3, mean3 = _range_stats(buf3.GetDIData())

            p2, min2, mean2 = 0, float("nan"), float("nan")
            buf2 = lidar_2d.GetMostRecentDIBuffer()
            if buf2.HasData():           # guard: skip frames the 2D lidar has not filled yet
                p2, min2, mean2 = _range_stats(buf2.GetDIData())

            data_writer.writerow([f"{t:.5f}", p3, f"{min3:.5f}", f"{mean3:.5f}",
                                  p2, f"{min2:.5f}", f"{mean2:.5f}"])

            pos = box_body.GetPos()
            vel = box_body.GetPosDt()
            motion_writer.writerow([f"{t:.5f}",
                                    f"{pos.x:.5f}", f"{pos.y:.5f}", f"{pos.z:.5f}",
                                    f"{vel.x:.5f}", f"{vel.y:.5f}", f"{vel.z:.5f}"])

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close partial CSV output even if a step diverges mid-run.
    if data_file is not None:
        data_file.close()
    if motion_file is not None:
        motion_file.close()

# === Post-processing === plot lidar range/point-count time series from the CSV
try:
    with open("simulation_data.csv", "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader if r]
except (OSError, IOError) as exc:   # missing/unreadable CSV after the run
    import traceback
    traceback.print_exc()
    rows = []

if rows:
    arr = np.array([[float(c) for c in r] for r in rows], dtype=float)
    t = arr[:, 0]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(t, arr[:, 1], label="3D lidar points")
    ax1.plot(t, arr[:, 4], label="2D lidar points")
    ax1.set_ylabel("valid points")
    ax1.legend(); ax1.grid(True)
    ax2.plot(t, arr[:, 3], label="3D mean range [m]")
    ax2.plot(t, arr[:, 6], label="2D mean range [m]")
    ax2.set_xlabel("time [s]"); ax2.set_ylabel("range [m]")
    ax2.legend(); ax2.grid(True)
    fig.suptitle("Dual lidar — point count and mean range vs time")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print("Simulation complete.")
