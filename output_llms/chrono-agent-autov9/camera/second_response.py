"""Orbiting RGB camera sensor over a static mesh object (PyChrono 9.0.x, Irrlicht + OptiX).

Model
-----
A single static, mesh-shaped object ("lime bunny") sits at the world origin on a
ground reference grid. An OptiX `ChCameraSensor` is attached to a fixed carrier
body via an offset pose of ChVector3d(-7, 0, 2) (7 m behind, 2 m above the target).
The camera is made to ORBIT the object: every physics step its offset pose is
rotated about the world +Z axis at a constant 0.1 rad/s, so the bunny is filmed
from a continuously sweeping azimuth. The camera saves 960x480 RGB frames.

System type
-----------
ChSystemNSC (Non-Smooth Contacts). Nothing actually collides or moves here — the
scene is static; the only "motion" is the camera's programmatic orbit. Collision
geometry is still attached to the visible object because the OptiX sensor renders
ONLY bodies that carry collision shapes.

Renderers
---------
- OptiX `ChCameraSensor` (the demo subject): produces the saved camera image stream.
- Irrlicht window (the standard review visualization): produces the review video.

Expected behavior
------------------
A stationary bunny mesh, lit by sensor-scene point lights, viewed by a camera that
slowly circles it. Camera world position traces a circle of radius ~7 m about the
origin; the bunny stays centered in frame.
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

# === Named constants === geometry / physics / camera parameters (no bare literals downstream)
TIME_STEP = 2.0e-3           # s, integration step
SIM_END = 12.0               # s, total simulated time
RENDER_FPS = 30.0            # Hz, Irrlicht review-frame cadence

IMAGE_WIDTH = 960            # px, camera image width
IMAGE_HEIGHT = 480           # px, camera image height
CAMERA_HFOV = 1.408          # rad, horizontal field of view
CAMERA_UPDATE_RATE = 30.0    # Hz, sensor tick rate (one saved camera frame per ~1/30 s)

ORBIT_RATE = 0.1             # rad/s, azimuthal orbit speed of the camera offset pose
CAM_OFFSET = chrono.ChVector3d(-7, 0, 2)   # camera offset pose relative to carrier body

OBJECT_DENSITY = 1000.0      # kg/m^3, density of the mesh object
MESH_SCALE = 18.0            # scale factor: enlarge the small source mesh so it fills the frame at 7 m
SAVE_IMAGES = True           # save camera frames to disk

MESH_FILE = "models/lime_bunny.obj"   # Chrono data-relative mesh asset

# Sensor-scene lighting positions (point lights illuminate the OptiX render).
LIGHT_POSITIONS = [
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChVector3f(-2, -2.5, 100),
]

# === Derived constants === precomputed once, never recomputed in the loop
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
ORBIT_PER_STEP = ORBIT_RATE * TIME_STEP                        # precomputed once: rad advanced per step
FRAME_SAVE_DIR = "cam/orbit_cam/"   # where ChFilterSave writes camera PNGs

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run

# === System & gravity === NSC system; static scene, gravity present but nothing falls
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies === one static mesh object at the origin, with both visual and collision geometry
contact_material = chrono.ChContactMaterialNSC()   # NSC material to match ChSystemNSC
contact_material.SetFriction(0.6)
contact_material.SetRestitution(0.0)

# Load the source mesh and scale it up: the raw asset is ~0.15 m, too small to read
# at the 7 m orbit radius, so enlarge it to a frame-filling size.
mesh_path = chrono.GetChronoDataFile(MESH_FILE)
tri_mesh = chrono.ChTriangleMeshConnected()
tri_mesh.LoadWavefrontMesh(mesh_path, True, True)   # load + merge duplicate vertices
tri_mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(MESH_SCALE))   # uniform scale

# Recenter the mesh on its bounding-box center: the raw bunny is offset from its
# local origin (its centroid sits ~2 m off-axis after scaling), which would make it
# swing across the frame as the camera orbits. Translating by -center puts the
# object's geometric center exactly at the body origin so the orbit stays centered.
mesh_center = tri_mesh.GetBoundingBox().Center()   # cache: computed once, reused below
tri_mesh.Transform(-mesh_center, chrono.ChMatrix33d(1.0))   # shift centroid to origin

mesh_object = chrono.ChBody()
mesh_object.SetName("orbit_target")
mesh_object.SetPos(chrono.ChVector3d(0, 0, 1.2))   # lift so the centered bunny rests above the grid
mesh_object.SetFixed(True)   # static target — only the camera moves

# Visual shape (seen by Irrlicht AND OptiX once a collision shape coexists).
visual_shape = chrono.ChVisualShapeTriangleMesh()
visual_shape.SetMesh(tri_mesh)
visual_shape.SetColor(chrono.ChColor(0.2, 0.8, 0.2))   # green bunny
mesh_object.AddVisualShape(visual_shape)

# Collision shape (REQUIRED: OptiX renders ONLY bodies that carry collision geometry).
coll_shape = chrono.ChCollisionShapeTriangleMesh(
    contact_material, tri_mesh, True, True, 0.005)
mesh_object.AddCollisionShape(coll_shape)
mesh_object.EnableCollision(True)
sys.Add(mesh_object)

# Target point the camera looks at (geometric center of the main body).
look_target = mesh_object.GetPos()   # cache: object is fixed, position never changes

# === Sensor manager & lighting === OptiX scene needs explicit lights (no AddDirectionalLight in 9.0.1)
manager = sens.ChSensorManager(sys)
for light_pos in LIGHT_POSITIONS:
    manager.scene.AddPointLight(light_pos, chrono.ChColor(1.0, 1.0, 1.0), 1000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))   # fill light for the sensor scene

# === Camera sensor === attached to a fixed carrier body; offset pose orbits the target
cam_body = chrono.ChBody()
cam_body.SetFixed(True)
cam_body.SetPos(chrono.ChVector3d(0, 0, 0))   # carrier at origin; the OFFSET places the lens
sys.Add(cam_body)

# Build the initial offset frame so the camera at CAM_OFFSET looks toward the target.
forward0 = (look_target - CAM_OFFSET).GetNormalized()
look_at_quat0 = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward0)
initial_offset = chrono.ChFramed(CAM_OFFSET, look_at_quat0)

camera = sens.ChCameraSensor(
    cam_body,            # carrier body
    CAMERA_UPDATE_RATE,  # Hz
    initial_offset,      # offset pose on the carrier
    IMAGE_WIDTH,         # width (px)
    IMAGE_HEIGHT,        # height (px)
    CAMERA_HFOV,         # horizontal FOV (rad)
)
camera.SetName("orbit_camera")
camera.PushFilter(sens.ChFilterVisualize(IMAGE_WIDTH, IMAGE_HEIGHT))   # live preview window
if SAVE_IMAGES:
    camera.PushFilter(sens.ChFilterSave(FRAME_SAVE_DIR))   # save PNG frames -> mp4 by RUN stage
camera.PushFilter(sens.ChFilterRGBA8Access())              # frame-buffer access
manager.AddSensor(camera)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid (review video)
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Orbiting camera over mesh object")
    vis.Initialize()                                     # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-7, 0, 2), chrono.ChVector3d(0, 0, 1))   # AFTER Initialize; matches sensor offset
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 40, 40,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid

# === Output dirs === guard against missing output directories
os.makedirs("frames", exist_ok=True)        # guard against missing Irrlicht-frame dir
os.makedirs(FRAME_SAVE_DIR, exist_ok=True)   # guard against missing sensor-frame dir
os.makedirs("cam", exist_ok=True)            # guard against missing motion-log dir

run_end = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

# === Main loop === render-cadence outer loop; orbit + sensor update per physics step
data_file = None
motion_file = None
data_writer = None
motion_writer = None
try:
    # with open guarantees both CSV writers flush/close even if a step diverges.
    with open("simulation_data.csv", "w", newline="") as data_file, \
         open("cam/motion_log.csv", "w", newline="") as motion_file:
        data_writer = csv.writer(data_file)
        data_writer.writerow(["time", "orbit_angle_rad", "cam_x", "cam_y", "cam_z", "cam_dist"])
        motion_writer = csv.writer(motion_file)
        motion_writer.writerow(["time", "body", "x", "y", "z", "cam_x", "cam_y", "cam_z"])

        orbit_angle = 0.0   # accumulated azimuth of the camera offset pose
        frame = 0
        while (HEADLESS or vis.Run()) and sys.GetChTime() < run_end:
            if not HEADLESS:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index -> ffmpeg
                frame += 1

            for _ in range(RENDER_EVERY):
                # Advance the camera orbit: rotate the base offset about world +Z.
                orbit_angle += ORBIT_PER_STEP
                orbit_q = chrono.QuatFromAngleZ(orbit_angle)
                cam_pos = orbit_q.Rotate(CAM_OFFSET)             # orbited camera position (world)
                forward = (look_target - cam_pos).GetNormalized()
                look_q = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)
                camera.SetOffsetPose(chrono.ChFramed(cam_pos, look_q))

                manager.Update()   # pump the sensor EVERY physics step (sees post-step pose)

                t = sys.GetChTime()
                dist = cam_pos.Length()
                data_writer.writerow([t, orbit_angle,
                                      cam_pos.x, cam_pos.y, cam_pos.z, dist])
                bp = mesh_object.GetPos()
                motion_writer.writerow([t, "orbit_camera",
                                        bp.x, bp.y, bp.z,
                                        cam_pos.x, cam_pos.y, cam_pos.z])

                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= run_end:
                    break

        # Guard sensor-buffer access: the buffer is empty before the first sensor tick.
        rgba_buffer = camera.GetMostRecentRGBA8Buffer()   # may be empty before first update
        if rgba_buffer.HasData():                          # guard: only read a filled buffer
            print(f"camera buffer: {rgba_buffer.Width}x{rgba_buffer.Height}")
except (OSError, IOError) as exc:        # disk / permission errors opening or writing CSVs
    import traceback
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Context managers above already flushed/closed the CSVs; nothing left open here.
    print("simulation loop finished; CSV writers closed")

# === Post-processing === plot camera trajectory vs time from the logged CSV
try:
    times, angles, cxs, cys, czs, dists = [], [], [], [], [], []
    with open("simulation_data.csv", "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            angles.append(float(row["orbit_angle_rad"]))
            cxs.append(float(row["cam_x"]))
            cys.append(float(row["cam_y"]))
            czs.append(float(row["cam_z"]))
            dists.append(float(row["cam_dist"]))

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(times, angles)
    axes[0].set_xlabel("time (s)"); axes[0].set_ylabel("orbit angle (rad)")
    axes[0].set_title("Camera orbit angle")
    axes[1].plot(cxs, cys)
    axes[1].set_xlabel("cam x (m)"); axes[1].set_ylabel("cam y (m)")
    axes[1].set_aspect("equal", adjustable="datalim")
    axes[1].set_title("Camera ground track (orbit)")
    axes[2].plot(times, dists)
    axes[2].set_xlabel("time (s)"); axes[2].set_ylabel("distance to origin (m)")
    axes[2].set_title("Camera radius (should stay ~constant)")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)
    print("wrote simulation_timeseries.png")
except (OSError, IOError) as exc:        # plotting / file read failure must not mask sim success
    import traceback
    traceback.print_exc()
