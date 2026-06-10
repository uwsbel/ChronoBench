"""Camera-sensor demo: an RGB camera rides on a textured box.

Model
-----
A single rigid cube (chrono.ChBodyEasyBox) is the protagonist. The cube carries
both a visual shape and collision geometry so it is visible to the OptiX sensor
renderer (which only draws bodies that own collision geometry). An RGB camera
sensor (sens.ChCameraSensor) is rigidly attached to the cube through an offset
pose of (-7, 0, 3) meters in the cube's local frame, looking back toward the
cube center. As the cube settles under gravity onto the ground, the onboard
camera records the scene.

System
------
- chrono.ChSystemNSC (non-smooth rigid contact), Z-up world, gravity (0,0,-9.81).
- Bodies: a fixed ground box and a free cube of side `BOX_SIDE` and density 1000.
- Sensor: a ChCameraSensor attached to the cube via a local offset frame; it
  emits PNG frames through ChFilterSave and a live preview through
  ChFilterVisualize.
- Visualization: an Irrlicht window (the standard review renderer) plus the
  OptiX camera sensor (the demo's actual subject). Both are driven from one
  explicit render-cadence loop.

Expected behavior
-----------------
The cube is released slightly above the ground, drops a short distance, and
comes to rest. The onboard camera, offset behind and above the cube, keeps the
cube centered in frame throughout. Per-step pose/velocity is logged to CSV and
plotted to a timeseries PNG.
"""

import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants === geometry, physics, timing, camera (no bare literals downstream)
TIME_STEP = 2e-3                 # s, integration step
SIM_END = 6.0                    # s, simulated duration
RENDER_FPS = 30.0                # Hz, Irrlicht review-frame cadence

BOX_SIDE = 4.0                   # m, cube edge length ("side")
BOX_DENSITY = 1000.0             # kg/m^3, cube material density
GROUND_SIZE = 40.0               # m, ground patch edge length
GROUND_THICK = 1.0               # m, ground patch thickness

DROP_HEIGHT = 0.5                # m, gap between cube bottom and ground at release
FRICTION = 0.6                   # contact friction coefficient
RESTITUTION = 0.0                # contact restitution (no bounce)

CAM_W, CAM_H = 1280, 720         # px, camera image resolution
CAM_HFOV = 1.408                 # rad, horizontal field of view
CAM_OFFSET = chrono.ChVector3d(-7.0, 0.0, 3.0)   # camera offset pose on the cube (local frame)

# Derived constants — precomputed once, never recomputed in the loop.
GROUND_TOP_Z = GROUND_THICK / 2.0                          # precomputed once
BOX_START_Z = GROUND_TOP_Z + BOX_SIDE / 2.0 + DROP_HEIGHT  # precomputed once: cube release height
CAM_UPDATE_RATE = 1.0 / TIME_STEP                          # precomputed once: sensor Hz
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once: steps per frame

HEADLESS = bool(os.environ.get("SIMBENCH_VALIDATE"))   # fast, windowless validation run
RUN_END = min(SIM_END, 0.5) if HEADLESS else SIM_END   # short physics check when validating

# === System & gravity === NSC rigid-contact system, Z-up, standard gravity
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared NSC material for ground + cube
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(FRICTION)
contact_mat.SetRestitution(RESTITUTION)

# === Bodies === fixed ground patch + free textured cube (both with collision -> visible to OptiX)
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                              BOX_DENSITY, True, True, contact_mat)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
ground_tex = chrono.ChVisualMaterial()
ground_tex.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.GetVisualShape(0).SetMaterial(0, ground_tex)
sys.Add(ground)

box = chrono.ChBodyEasyBox(BOX_SIDE, BOX_SIDE, BOX_SIDE,
                           BOX_DENSITY, True, True, contact_mat)
box.SetPos(chrono.ChVector3d(0, 0, BOX_START_Z))
box_tex = chrono.ChVisualMaterial()
box_tex.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))
box.GetVisualShape(0).SetMaterial(0, box_tex)
sys.Add(box)

# === Sensor camera === ChCameraSensor rigidly attached to the cube via a local offset frame
manager = sens.ChSensorManager(sys)
# 9.0.1 quirk: ChScene exposes AddPointLight + SetAmbientLight (no AddDirectionalLight).
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-20, 20, 30), chrono.ChColor(1, 1, 1), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# Look-at: camera local +X is its view axis; point it from the offset back to the cube center.
cam_forward = (chrono.ChVector3d(0, 0, 0) - CAM_OFFSET).GetNormalized()
cam_look_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), cam_forward)
cam = sens.ChCameraSensor(
    box,                                                # rides on the cube -> follows it
    CAM_UPDATE_RATE,                                    # Hz
    chrono.ChFramed(CAM_OFFSET, cam_look_quat),         # offset pose (-7,0,3) on the cube
    CAM_W, CAM_H,
    CAM_HFOV,
)
cam.SetName("box_chase_cam")
cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H))    # live preview window
cam.PushFilter(sens.ChFilterSave("cam/box_chase_cam/")) # PNG frames -> mp4 by RUN stage
cam.PushFilter(sens.ChFilterRGBA8Access())              # frame-buffer access
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
if not HEADLESS:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Camera attached to box")
    vis.Initialize()                                    # Initialize FIRST (inverse of VSG)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-12, -12, 8), chrono.ChVector3d(0, 0, 2))
    vis.AddTypicalLights()
    vis.AddGrid(2.0, 2.0, 20, 20,
                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z), chrono.QUNIT),
                chrono.ChColor(0.4, 0.4, 0.4))

# === Output directories === guard against missing output dirs before opening writers
os.makedirs("frames", exist_ok=True)        # Irrlicht review frames
os.makedirs("cam", exist_ok=True)            # sensor frames + motion log

# === Main loop === render-cadence outer loop; physics + manager.Update() in inner batch
data_f = None
motion_f = None
try:
    try:
        data_f = open("simulation_data.csv", "w", newline="")        # disk / permission errors
        motion_f = open("cam/motion_log.csv", "w", newline="")
    except (OSError, IOError) as exc:                                 # cannot open CSV target
        print("Failed to open CSV output:", exc)
        raise

    data_w = csv.writer(data_f)
    data_w.writerow(["time", "box_x", "box_y", "box_z", "box_vz"])
    motion_w = csv.writer(motion_f)
    motion_w.writerow(["time", "body", "x", "y", "z", "vx", "vy", "vz"])

    times, zs, vzs = [], [], []
    frame = 0
    while (HEADLESS or vis.Run()) and sys.GetChTime() < RUN_END:
        if not HEADLESS:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.WriteImageToFile(f"frames/img_{frame:06d}.png")   # consecutive index for ffmpeg
            frame += 1
        for _ in range(RENDER_EVERY):
            manager.Update()                                      # pump sensor every physics step
            t = sys.GetChTime()
            p = box.GetPos()                                      # cube world position this step
            v = box.GetPosDt()                                    # cube world velocity this step
            data_w.writerow([t, p.x, p.y, p.z, v.z])
            motion_w.writerow([t, "box", p.x, p.y, p.z, v.x, v.y, v.z])
            times.append(t); zs.append(p.z); vzs.append(v.z)

            # Guard sensor buffer: empty until the first sensor tick.
            buf = cam.GetMostRecentRGBA8Buffer()                  # may be empty before first tick
            if buf.HasData():                                     # guard: skip unfilled frames
                _ = buf.GetRGBA8Data()                            # safe to read only after HasData()

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= RUN_END:
                break
except (RuntimeError, ValueError) as exc:        # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Flush + close any open writers even if a step diverged mid-run.
    if data_f is not None:
        data_f.close()
    if motion_f is not None:
        motion_f.close()

# === Post-processing === plot the logged cube motion vs time
if times:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, zs, color="tab:blue")
    ax1.set_ylabel("box z (m)")
    ax1.grid(True)
    ax2.plot(times, vzs, color="tab:red")
    ax2.set_ylabel("box vz (m/s)")
    ax2.set_xlabel("time (s)")
    ax2.grid(True)
    fig.suptitle("Box vertical motion (camera-attached cube)")
    fig.tight_layout()
    fig.savefig("simulation_timeseries.png", dpi=110)
    plt.close(fig)

print(f"Done: {len(times)} steps logged, final box z = {zs[-1] if zs else float('nan'):.4f} m")
