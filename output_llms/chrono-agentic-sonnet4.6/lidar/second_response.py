"""
Lidar sensor demonstration with a box object.

Models a rotating box body (ChBodyEasyBox) with:
  - A 3D lidar sensor attached to it (horizontal + vertical sweeps)
  - A 2D lidar sensor (single-row, 360-degree horizontal scan)

Both lidars collect depth/intensity data, visualize point clouds, and log
output via filter chains. System type: ChSystemNSC, Z-up gravity, Irrlicht
window for review. Expected behavior: the box rotates in place while the
lidars continuously scan the surrounding environment, visualizing point-cloud
data in real time.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
side = 1.0          # box half-side length (m)
BOX_DENSITY = 1000  # kg/m^3

# Lidar parameters — 3D lidar
LIDAR_UPDATE_RATE = 5.0       # Hz (physical rate)
H_SAMPLES = 800
V_SAMPLES = 300
H_FOV = 2 * chrono.CH_PI
MAX_VERT = chrono.CH_PI / 12
MIN_VERT = -chrono.CH_PI / 6
MAX_RANGE = 100.0

# 2D lidar parameters
LIDAR_2D_UPDATE_RATE = 10.0   # Hz
H_SAMPLES_2D = 800
V_SAMPLES_2D = 1   # single horizontal row

render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Ground floor
mat_ground = chrono.ChContactMaterialNSC()
mat_ground.SetFriction(0.8)
mat_ground.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 0.2, 1000, True, True, mat_ground)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.SetFixed(True)
sys.AddBody(ground)

# Box object (replaces a mesh — box with specified side dimension)
mat_box = chrono.ChContactMaterialNSC()
mat_box.SetFriction(0.5)
mat_box.SetRestitution(0.0)
box_body = chrono.ChBodyEasyBox(side, side, side, BOX_DENSITY, True, True, mat_box)
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5 + side / 2.0))
box_body.SetFixed(False)

# Apply texture to box
vis_box = box_body.GetVisualModel().GetShape(0)
vis_box.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.AddBody(box_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
# Point lights for camera-based sensors (camera only — lidar needs no lighting)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === 3D Lidar sensor attached to box ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    box_body,                                # attach to box body
    LIDAR_UPDATE_RATE,                       # physical update rate (Hz)
    lidar_offset,                            # offset pose
    H_SAMPLES,                               # horizontal samples
    V_SAMPLES,                               # vertical samples
    H_FOV,                                   # horizontal FOV (full 360)
    MAX_VERT,                                # max vertical angle
    MIN_VERT,                                # min vertical angle
    MAX_RANGE,                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,         # beam shape
    2,                                       # sample radius
    0.003,                                   # vertical divergence angle
    0.003,                                   # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,   # return mode
)
lidar.SetName("3D Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)  # collection window = 1/update_rate

# 3D lidar filter chain
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())             # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())          # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())           # host access to XYZI
manager.AddSensor(lidar)

# === 2D Lidar sensor (single horizontal row) ===
lidar_2d_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    box_body,                                # attach to box body
    LIDAR_2D_UPDATE_RATE,                    # physical update rate (Hz)
    lidar_2d_offset,                         # offset pose
    H_SAMPLES_2D,                            # horizontal samples
    V_SAMPLES_2D,                            # 1 vertical channel => 2D lidar
    H_FOV,                                   # horizontal FOV (full 360)
    0,                                       # max_vert_angle = 0 (2D)
    0,                                       # min_vert_angle = 0 (2D)
    MAX_RANGE,                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_2D_UPDATE_RATE)

# 2D lidar filter chain
lidar_2d.PushFilter(sens.ChFilterVisualize(H_SAMPLES_2D, V_SAMPLES_2D, "Raw 2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar_2d)

# === Visualization — full Irrlicht scene ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Sensor Demo — Box + 3D/2D Lidar")
vis.Initialize()                                       # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()  # pump sensors exactly once per physics step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
