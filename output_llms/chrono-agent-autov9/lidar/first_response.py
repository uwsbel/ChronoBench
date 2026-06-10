"""Lidar-on-a-mesh sensing simulation (PyChrono 9.0.1, NSC rigid system).

What this models
----------------
A single triangular Wavefront ``.obj`` mesh (the Stanford bunny shipped in the
Chrono data set) is loaded as a fixed rigid body at the world origin. The mesh
is given BOTH a visual triangle mesh (for the Irrlicht review window) AND a
triangle-mesh collision shape, because the OptiX-based lidar ray caster only
returns hits from bodies that carry collision geometry.

A rotating ``ChLidarSensor`` orbits the mesh: a kinematic carrier body is driven
analytically in a circle around the origin while always aiming its lidar at the
mesh centre. The lidar runs through the canonical filter chain
(depth/intensity access -> point cloud from depth -> point-cloud visualization
-> point-cloud save -> XYZI access). Beam divergence and a multi-sample beam
radius emulate sensor noise/spread. Every simulation step the most-recent lidar
depth/intensity buffer is read, its statistics (returned-point count, min / mean
range, mean intensity) are printed and logged to ``simulation_data.csv``; the
orbiting carrier pose is logged to ``cam/motion_log.csv``.

System type: ``ChSystemNSC`` (rigid, gravity off — the only motion is the
prescribed kinematic orbit of the lidar carrier, so contact dynamics are not
needed). Expected behaviour: the lidar continuously reports valid ranges to the
mesh surface as it circles it, and the printed/CSV range statistics stay bounded
between the clip-near distance and the configured max range.
"""

# === Imports ===
import os
import csv
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless-safe backend for the timeseries PNG
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants (geometry / physics / sensor) ===
TIME_STEP = 4.0e-3            # s, physics integration step
SIM_END = 6.0                 # s, total simulated duration (one+ full orbit)
RENDER_FPS = 20.0             # Hz, Irrlicht review-frame cadence

ORBIT_RADIUS = 3.5            # m, lidar carrier orbit radius around the mesh
ORBIT_HEIGHT = 1.2            # m, lidar carrier height above ground
ORBIT_PERIOD = 6.0            # s, time for one full revolution
MESH_SCALE = 10.0            # uniform up-scale of the small stock bunny mesh -> visible size
MESH_CENTER = chrono.ChVector3d(0.0, 0.0, 1.0)  # aim point on the mesh

LIDAR_UPDATE_RATE = 10.0      # Hz, lidar scan rate
LIDAR_W = 400                 # horizontal samples (beams per scan line)
LIDAR_H = 120                 # vertical samples (scan lines)
LIDAR_HFOV = 2.0 * math.pi    # rad, full 360 deg horizontal field of view
LIDAR_MAX_VERT = 0.20         # rad, upper vertical beam angle
LIDAR_MIN_VERT = -0.20        # rad, lower vertical beam angle
LIDAR_MAX_DIST = 100.0        # m, maximum return distance
LIDAR_SAMPLE_RADIUS = 2       # multi-sample beam radius (emulates beam spread/noise)
LIDAR_DIVERGENCE = 0.003      # rad, beam divergence angle (vertical & horizontal)
LIDAR_CLIP_NEAR = 1e-2        # m, near clip distance

VIS_W, VIS_H = 1280, 720      # Irrlicht review window size

# Derived constants (precomputed once — never recomputed in the hot loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
ORBIT_OMEGA = 2.0 * math.pi / ORBIT_PERIOD                    # precomputed once, rad/s
MESH_FILE = chrono.GetChronoDataFile("models/lime_bunny.obj")  # triangular .obj asset

# Headless validation gate: a fast, windowless physics+sensor check.
HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))  # fast, windowless validation run


def orbit_pose(t):
    """Return (position, look-at quaternion) of the lidar carrier at time ``t``.

    The carrier rides a circle of radius ORBIT_RADIUS at height ORBIT_HEIGHT and
    always points its local +X axis (the lidar's view axis) toward MESH_CENTER.
    """
    angle = ORBIT_OMEGA * t
    pos = chrono.ChVector3d(
        ORBIT_RADIUS * math.cos(angle),
        ORBIT_RADIUS * math.sin(angle),
        ORBIT_HEIGHT,
    )
    forward = (MESH_CENTER - pos).GetNormalized()
    look_at = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)
    return pos, look_at


# === System & gravity ===
# Rigid NSC world; gravity disabled because the only motion is the prescribed
# kinematic lidar orbit (no free-fall / contact dynamics to resolve).
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies (fixed mesh target + kinematic lidar carrier) ===
# Contact material (NSC) shared by the mesh collision shape.
mesh_mat = chrono.ChContactMaterialNSC()
mesh_mat.SetFriction(0.6)
mesh_mat.SetRestitution(0.0)

# Load the triangular mesh once and reuse it for both visual and collision shapes.
# The stock bunny is ~0.15 m; uniformly scale it up so it reads clearly in the
# review window and presents a substantial surface to the orbiting lidar.
trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(MESH_FILE, True, True)  # load_normals, load_uv
trimesh.Transform(chrono.ChVector3d(0, 0, 0),
                  chrono.ChMatrix33d(MESH_SCALE))  # uniform scale about origin

mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)                 # static target in the scene
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))

vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh.SetMesh(trimesh)
vis_mesh.SetName("bunny_mesh")
vis_mesh.SetColor(chrono.ChColor(0.3, 0.7, 0.3))
mesh_body.AddVisualShape(vis_mesh, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Collision triangle mesh so the OptiX lidar actually returns hits off the mesh.
coll_mesh = chrono.ChCollisionShapeTriangleMesh(mesh_mat, trimesh, True, True, 0.0)
mesh_body.AddCollisionShape(coll_mesh, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
mesh_body.EnableCollision(True)
sys.Add(mesh_body)

# Add a large fixed ground box (visual + collision) for a stable lidar floor return
# and an Irrlicht ground reference.
ground = chrono.ChBodyEasyBox(20.0, 20.0, 0.2, 1000.0, True, True, mesh_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.SetFixed(True)
sys.Add(ground)

# Kinematic carrier the lidar rides on; its pose is set analytically each step.
lidar_carrier = chrono.ChBody()
lidar_carrier.SetFixed(False)
init_pos, init_quat = orbit_pose(0.0)
lidar_carrier.SetPos(init_pos)
lidar_carrier.SetRot(init_quat)
sys.Add(lidar_carrier)

# === Sensor manager & lighting (OptiX scene for the lidar) ===
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1.0, 1.0, 1.0), 500.0
)
# 9.0.1 quirk: ChScene has no AddDirectionalLight -> use point light + ambient.
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# === Lidar sensor (canonical filter chain + beam-spread noise emulation) ===
lidar = sens.ChLidarSensor(
    lidar_carrier,                                   # body the lidar rides on
    LIDAR_UPDATE_RATE,                               # Hz scan rate
    chrono.ChFramed(chrono.VNULL, chrono.QUNIT),     # mounted at carrier origin, +X view axis
    LIDAR_W, LIDAR_H,                                # horizontal / vertical samples
    LIDAR_HFOV,                                      # horizontal FOV (rad)
    LIDAR_MAX_VERT, LIDAR_MIN_VERT,                  # vertical beam extents (rad)
    LIDAR_MAX_DIST,                                  # max return distance (m)
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_RADIUS,                             # multi-sample beam radius (spread/noise)
    LIDAR_DIVERGENCE, LIDAR_DIVERGENCE,              # vert / hori divergence (rad)
    sens.LidarReturnMode_MEAN_RETURN,                # average multi-sample returns
    LIDAR_CLIP_NEAR,
)
lidar.SetName("orbit_lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Filter chain: depth/intensity access -> XYZ point cloud -> visualize -> save -> XYZI access.
lidar.PushFilter(sens.ChFilterDIAccess())                          # raw depth+intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())                       # build XYZ point cloud
# The point cloud is persisted to disk every scan; the heavy live GL point-cloud
# preview window is omitted so the orbit renders within the wall-clock budget.
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_pc/"))            # save point clouds to disk
lidar.PushFilter(sens.ChFilterXYZIAccess())                        # processed XYZI access
manager.AddSensor(lidar)

# === Visualization (full Irrlicht scene: window + sky + camera + lights + grid) ===
# Built only for the on-screen review run; the headless gate skips the window for
# a fast validation pass but the complete setup remains in the source.
vis = None
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(VIS_W, VIS_H)
    vis.SetWindowTitle("Orbiting Lidar over Triangular Mesh")
    vis.Initialize()  # Initialize FIRST, then add scene elements (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.5, -4.5, 3.0), chrono.ChVector3d(0, 0, 1.0))
    vis.AddTypicalLights()
    vis.AddGrid(
        0.5, 0.5, 40, 40,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        chrono.ChColor(0.4, 0.4, 0.4),
    )

# === Main loop (render-cadence outer loop; sensor + CSV per physics step) ===
run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END  # short physics check when validating
os.makedirs("frames", exist_ok=True)  # guard against missing review-frame output dir
os.makedirs("cam", exist_ok=True)     # guard against missing motion-log output dir

# Cached handles fetched once and reused every step.
get_lidar = lidar              # cache: sensor handle reused every step
carrier = lidar_carrier        # cache: carrier body reused every step

times = []
pt_counts = []
min_ranges = []
mean_ranges = []
mean_intensities = []

data_f = None
motion_f = None
try:
    # Guard the file opens specifically (disk full / permission errors).
    try:
        data_f = open("simulation_data.csv", "w", newline="")
        motion_f = open(os.path.join("cam", "motion_log.csv"), "w", newline="")
    except (OSError, IOError) as exc:  # disk / permission failure opening CSV
        raise RuntimeError(f"could not open output CSV: {exc}")

    data_w = csv.writer(data_f)
    data_w.writerow(
        ["time", "lidar_point_count", "min_range_m", "mean_range_m", "mean_intensity"]
    )
    motion_w = csv.writer(motion_f)
    motion_w.writerow(["time", "carrier_x", "carrier_y", "carrier_z", "speed_mps"])

    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")  # consecutive index -> ffmpeg
            frame += 1

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # Prescribe the kinematic orbit pose of the lidar carrier this step.
            pos, look_at = orbit_pose(t)
            carrier.SetPos(pos)
            carrier.SetRot(look_at)
            speed = ORBIT_OMEGA * ORBIT_RADIUS  # tangential speed (constant)
            carrier.SetPosDt(chrono.ChVector3d(
                -ORBIT_OMEGA * ORBIT_RADIUS * math.sin(ORBIT_OMEGA * t),
                ORBIT_OMEGA * ORBIT_RADIUS * math.cos(ORBIT_OMEGA * t),
                0.0,
            ))

            # Pump the sensor every physics step so it sees the latest pose.
            manager.Update()

            # Read the most-recent depth/intensity buffer; guard before access.
            di_buf = get_lidar.GetMostRecentDIBuffer()  # may be empty before first tick
            n_pts = 0
            r_min = float("nan")
            r_mean = float("nan")
            i_mean = float("nan")
            if di_buf.HasData():  # guard: skip frames the sensor has not filled yet
                di = di_buf.GetDIData()  # numpy array shape (H, W, 2): [range, intensity]
                ranges = di[:, :, 0]
                intens = di[:, :, 1]
                valid = (ranges > LIDAR_CLIP_NEAR) & (ranges < LIDAR_MAX_DIST)
                n_pts = int(np.count_nonzero(valid))
                if n_pts > 0:
                    r_min = float(np.min(ranges[valid]))
                    r_mean = float(np.mean(ranges[valid]))
                    i_mean = float(np.mean(intens[valid]))

            print(
                f"t={t:6.3f}s  lidar points={n_pts:6d}  "
                f"min_range={r_min:8.3f}  mean_range={r_mean:8.3f}  mean_int={i_mean:7.4f}"
            )

            times.append(t)
            pt_counts.append(n_pts)
            min_ranges.append(r_min)
            mean_ranges.append(r_mean)
            mean_intensities.append(i_mean)
            data_w.writerow([f"{t:.5f}", n_pts, r_min, r_mean, i_mean])
            motion_w.writerow([f"{t:.5f}", pos.x, pos.y, pos.z, speed])

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= run_end:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverges mid-run.
    if data_f is not None:
        data_f.close()
    if motion_f is not None:
        motion_f.close()

# === Post-processing (timeseries plot from the logged CSV columns) ===
if times:
    t_arr = np.array(times)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(t_arr, pt_counts, color="tab:blue", label="lidar point count")
    ax1.set_ylabel("returned points")
    ax1.grid(True)
    ax1.legend(loc="best")
    ax2.plot(t_arr, min_ranges, color="tab:red", label="min range [m]")
    ax2.plot(t_arr, mean_ranges, color="tab:green", label="mean range [m]")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("range [m]")
    ax2.grid(True)
    ax2.legend(loc="best")
    fig.suptitle("Orbiting lidar — returns and range statistics")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print("Simulation complete.")
