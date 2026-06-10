"""Orbiting camera sensor observing a fixed triangular mesh.

Model
-----
A single triangular surface mesh is loaded from a Wavefront ``.obj`` file
(``models/lime_bunny.obj`` from the Chrono data set) and placed as a *fixed*
rigid body at the world origin. The body carries both a visual triangle-mesh
shape (so the Irrlicht review window draws it) and a triangle-mesh collision
shape (so the OptiX sensor renderer, which only sees collision geometry, can
also image it).

System type: ``ChSystemNSC`` (non-smooth contact). Nothing moves under
dynamics here — the mesh is fixed and there is no contact event — so the
solver simply advances time while the *camera viewpoint* is driven
kinematically.

Sensor
------
A ``ChSensorManager`` owns one ``ChCameraSensor`` mounted on a small camera
carrier body. Two camera-noise filters (constant-normal additive noise and a
pixel-dependent noise model) plus a live ``ChFilterVisualize`` window and a
``ChFilterSave`` PNG dump are applied to the camera image stream, and an
``RGBA8`` access filter exposes the frame buffer so its statistics can be
printed every step.

Expected behavior
-----------------
The carrier body (and therefore the camera) is moved each physics step along a
circular orbit at fixed radius and height, always looking at the mesh centre.
The recorded frames sweep around the bunny; the printed buffer data shows a
non-empty RGBA frame once the sensor has produced its first image.
"""

import csv
import math
import os

import matplotlib
matplotlib.use("Agg")  # headless plotting backend — no display needed
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants (geometry / physics / camera orbit) ===
TIME_STEP = 2.0e-3          # s, integration step
SIM_END = 6.0               # s, total simulated time
RENDER_FPS = 30.0           # Irrlicht review-frame cadence

MESH_FILE = "models/lime_bunny.obj"   # Wavefront triangular mesh (Chrono data)
MESH_POS = chrono.ChVector3d(0.0, 0.0, 0.0)   # fixed mesh centre at world origin

ORBIT_RADIUS = 4.0          # m, camera distance from mesh centre
ORBIT_HEIGHT = 1.5          # m, camera height above the mesh centre
ORBIT_PERIOD = 6.0          # s, time for one full revolution

CAM_W, CAM_H = 1280, 720    # sensor image resolution (pixels)
CAM_HFOV = 1.408            # rad, horizontal field of view
CAM_UPDATE_RATE = 1.0 / TIME_STEP   # Hz, sensor update rate == physics rate

# Derived constants — computed ONCE before the loop (never recomputed inside it).
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
ORBIT_OMEGA = 2.0 * math.pi / ORBIT_PERIOD                     # precomputed once: rad/s
WORLD_UP = chrono.ChVector3d(0, 0, 1)                          # precomputed once

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run


def orbit_position(t):
    """Camera world position on the circular orbit at simulated time ``t``."""
    angle = ORBIT_OMEGA * t
    return chrono.ChVector3d(
        MESH_POS.x + ORBIT_RADIUS * math.cos(angle),
        MESH_POS.y + ORBIT_RADIUS * math.sin(angle),
        MESH_POS.z + ORBIT_HEIGHT,
    )


def look_at_quat(eye, target):
    """Quaternion rotating local +X (camera view axis) onto eye->target."""
    forward = (target - eye).GetNormalized()
    return chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)


# === System & gravity === one NSC system owns the mesh, camera carrier, sensors.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # mesh collision -> visible to OptiX

# === Bodies === load the Wavefront triangular mesh once, attach visual + collision.
mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    chrono.GetChronoDataFile(MESH_FILE), True, True
)

mesh_body = chrono.ChBody()
mesh_body.SetPos(MESH_POS)
mesh_body.SetFixed(True)   # static scene object — never moves under dynamics

vshape = chrono.ChVisualShapeTriangleMesh()
vshape.SetMesh(mesh)
vshape.SetName("bunny_mesh")
vshape.SetColor(chrono.ChColor(0.7, 0.55, 0.35))
mesh_body.AddVisualShape(vshape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Collision geometry so the OptiX sensor camera (collision-only renderer) sees it.
mesh_mat = chrono.ChContactMaterialNSC()   # NSC material to match the NSC system
mesh_mat.SetFriction(0.6)
mesh_mat.SetRestitution(0.0)
coll = chrono.ChCollisionShapeTriangleMesh(mesh_mat, mesh, True, True, 0.005)
mesh_body.AddCollisionShape(coll, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
mesh_body.EnableCollision(True)
sys.AddBody(mesh_body)

mesh_center = mesh_body.GetPos()   # cache: fetched once, reused as the orbit look-at target

# Camera carrier body — kinematically driven around the orbit (not free-falling).
cam_body = chrono.ChBody()
cam_body.SetFixed(True)   # pose is set explicitly each step, not integrated
cam_body.SetPos(orbit_position(0.0))
sys.AddBody(cam_body)

# === Sensor manager & camera === noise + visualize + save + buffer-access filters.
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 100), chrono.ChColor(1.0, 1.0, 1.0), 1000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-10, -10, 40), chrono.ChColor(0.7, 0.7, 0.7), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.4, 0.4, 0.4))

_init_quat = look_at_quat(orbit_position(0.0), mesh_center)
camera = sens.ChCameraSensor(
    cam_body,                                          # rides on the orbiting carrier
    CAM_UPDATE_RATE,                                   # Hz
    chrono.ChFramed(chrono.VNULL, _init_quat),         # offset frame: look at mesh
    CAM_W, CAM_H,
    CAM_HFOV,
)
camera.SetName("orbit_camera")
camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))   # additive sensor noise
camera.PushFilter(sens.ChFilterCameraNoisePixDep(0.0001, 0.0001))   # pixel-dependent noise
camera.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))             # live preview window
camera.PushFilter(sens.ChFilterSave("cam/sensor_frames/"))          # PNG frames -> mp4 later
camera.PushFilter(sens.ChFilterRGBA8Access())                       # frame-buffer access
manager.AddSensor(camera)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid.
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Orbiting camera sensor over a triangular mesh")
    vis.Initialize()                                   # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(6, -6, 4), mesh_center)   # AFTER Initialize
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))         # ground reference grid

# === Main loop === orbit the camera, pump the sensor, log + print buffer data.
os.makedirs("frames", exist_ok=True)          # guard: Irrlicht review frames
os.makedirs("cam", exist_ok=True)             # guard: sensor PNGs + logs live here

run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

sim_csv = None
motion_csv = None
try:
    # with open(...) so writers always flush/close even on a mid-run error.
    with open("simulation_data.csv", "w", newline="") as sim_f, \
         open("cam/motion_log.csv", "w", newline="") as motion_f:
        sim_csv = csv.writer(sim_f)
        motion_csv = csv.writer(motion_f)
        sim_csv.writerow(["time", "cam_x", "cam_y", "cam_z",
                          "dist_to_mesh", "buf_has_data", "buf_mean_intensity"])
        motion_csv.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index
                frame += 1

            for _ in range(RENDER_EVERY):
                t = sys.GetChTime()

                # Drive the camera carrier kinematically around the mesh.
                eye = orbit_position(t)
                cam_body.SetPos(eye)
                cam_body.SetRot(look_at_quat(eye, mesh_center))

                manager.Update()   # pump the sensor EVERY physics step

                # Read the camera frame buffer (guarded — empty before first tick).
                buf = camera.GetMostRecentRGBA8Buffer()   # may be empty pre-first-update
                has_data = 0
                mean_intensity = 0.0
                if buf.HasData():                          # guard: skip unfilled frames
                    rgba = buf.GetRGBA8Data()              # safe only after HasData()
                    has_data = 1
                    mean_intensity = float(rgba.mean())    # numpy array of shape (H, W, 4)
                    print(f"t={t:6.3f}s  buffer {buf.Width}x{buf.Height}  "
                          f"mean_intensity={mean_intensity:7.3f}")
                else:
                    print(f"t={t:6.3f}s  buffer empty (sensor not yet produced a frame)")

                dist = (eye - mesh_center).Length()
                sim_csv.writerow([f"{t:.6f}", f"{eye.x:.6f}", f"{eye.y:.6f}",
                                  f"{eye.z:.6f}", f"{dist:.6f}", has_data,
                                  f"{mean_intensity:.6f}"])
                v = cam_body.GetPosDt()
                motion_csv.writerow([f"{t:.6f}", "orbit_camera",
                                     f"{eye.x:.6f}", f"{eye.y:.6f}", f"{eye.z:.6f}",
                                     f"{v.x:.6f}", f"{v.y:.6f}", f"{v.z:.6f}"])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break
except (OSError, IOError) as exc:          # disk / permission errors on the CSV files
    import traceback
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Writers are closed by the `with` block; nothing else to flush here.
    pass

# === Post-processing === plot logged camera-orbit time series from the CSV.
times, cam_x, cam_y, cam_z, dists, intens = [], [], [], [], [], []
with open("simulation_data.csv", "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["time"]))
        cam_x.append(float(row["cam_x"]))
        cam_y.append(float(row["cam_y"]))
        cam_z.append(float(row["cam_z"]))
        dists.append(float(row["dist_to_mesh"]))
        intens.append(float(row["buf_mean_intensity"]))

if times:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(times, cam_x, label="cam_x")
    ax1.plot(times, cam_y, label="cam_y")
    ax1.plot(times, cam_z, label="cam_z")
    ax1.plot(times, dists, "k--", label="dist_to_mesh")
    ax1.set_ylabel("position / distance [m]")
    ax1.legend(loc="upper right")
    ax1.set_title("Orbiting camera pose and mean image intensity")
    ax2.plot(times, intens, "r-", label="mean image intensity")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("mean RGBA intensity")
    ax2.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print("Done.")
