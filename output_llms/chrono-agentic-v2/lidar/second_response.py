"""
Lidar Sensor Demo — Box Object with 3D and 2D Lidar Sensors.

Models a fixed box body (ChBodyEasyBox) with two lidar sensors attached:
  1. A 3D lidar (multi-channel) for full volumetric point-cloud capture.
  2. A 2D lidar (single vertical channel) for planar scanning.

System: ChSystemNSC (rigid static scene, collision enabled for OptiX rendering).
Expected behavior: both lidar sensors spin and capture the box; point-cloud
visualizations appear for both sensors while Irrlicht renders the scene.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
TIME_STEP  = 1e-3          # physics integration step (s)
SIM_END    = 10.0          # simulation duration (s)
RENDER_FPS = 50.0          # Irrlicht render target (frames/s)

# Box geometry
SIDE = 2.0                 # box half-side length used for ChBodyEasyBox (full side)

# Lidar parameters (3D)
LIDAR_UPDATE_RATE  = 5.0            # Hz — physical rate
H_SAMPLES          = 800            # horizontal resolution
V_SAMPLES          = 300            # vertical channels
LIDAR_MAX_RANGE    = 100.0          # m
LIDAR_HFOV         = 2 * chrono.CH_PI   # full 360° horizontal sweep
LIDAR_MAX_VERT     = chrono.CH_PI / 12  # +15° vertical
LIDAR_MIN_VERT     = -chrono.CH_PI / 6  # -30° vertical

# Lidar parameters (2D)
LIDAR_2D_H_SAMPLES = 800
LIDAR_2D_V_SAMPLES = 1               # single channel → 2D
LIDAR_2D_MAX_VERT  = 0.0             # both angles = 0 for 2D
LIDAR_2D_MIN_VERT  = 0.0

render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Contact material for the box
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

# Box body — ChBodyEasyBox uses FULL side extents
box_body = chrono.ChBodyEasyBox(SIDE, SIDE, SIDE, 1000, True, True, mat)
box_body.SetPos(chrono.ChVector3d(0, 0, 1))
box_body.SetFixed(True)

# Add a texture to the box for visual contrast
box_texture = chrono.ChVisualMaterial()
box_texture.SetDiffuseColor(chrono.ChColor(0.4, 0.6, 0.8))
box_body.GetVisualShape(0).SetMaterial(0, box_texture)

sys.AddBody(box_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
# Point light for OptiX camera (if added); lidar itself doesn't need lights
# but a point light keeps the scene well-lit for the Irrlicht review window too
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# === 3D Lidar sensor (attached to box_body) ===
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    box_body,                                    # body the lidar is attached to
    LIDAR_UPDATE_RATE,                           # update rate (Hz)
    lidar_offset_pose,                           # offset pose
    H_SAMPLES,                                   # horizontal samples
    V_SAMPLES,                                   # vertical samples
    LIDAR_HFOV,                                  # horizontal FOV (rad)
    LIDAR_MAX_VERT,                              # max vertical angle (rad)
    LIDAR_MIN_VERT,                              # min vertical angle (rad)
    LIDAR_MAX_RANGE,                             # max range (m)
    sens.LidarBeamShape_RECTANGULAR,             # beam shape
    2,                                           # sample_radius
    0.003,                                       # vertical divergence angle
    0.003,                                       # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,       # return mode
)
lidar.SetName("3D Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

# 3D lidar filter chain
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())               # depth+intensity host access
lidar.PushFilter(sens.ChFilterPCfromDepth())            # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())             # XYZI host access
manager.AddSensor(lidar)

# === 2D Lidar sensor (single vertical channel, attached to box_body) ===
lidar_2d_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    box_body,                                    # body the 2D lidar is attached to
    LIDAR_UPDATE_RATE,                           # update rate (Hz)
    lidar_2d_offset_pose,                        # offset pose
    LIDAR_2D_H_SAMPLES,                          # horizontal samples
    LIDAR_2D_V_SAMPLES,                          # v_samples = 1 → 2D lidar
    LIDAR_HFOV,                                  # horizontal FOV (rad)
    LIDAR_2D_MAX_VERT,                           # max vertical angle = 0 for 2D
    LIDAR_2D_MIN_VERT,                           # min vertical angle = 0 for 2D
    LIDAR_MAX_RANGE,                             # max range (m)
    sens.LidarBeamShape_RECTANGULAR,             # beam shape
    2,                                           # sample_radius
    0.003,                                       # vertical divergence angle
    0.003,                                       # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,       # return mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

# 2D lidar filter chain
lidar_2d.PushFilter(sens.ChFilterVisualize(LIDAR_2D_H_SAMPLES, LIDAR_2D_V_SAMPLES, "2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())            # depth+intensity host access
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())         # depth → XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())          # XYZI host access
manager.AddSensor(lidar_2d)

# === Visualization (full Irrlicht block) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Demo — Box with 3D and 2D Lidar")
vis.Initialize()                                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 1))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only: recording setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                    # pump sensors every physics step
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:       # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass                                         # ensures finally is non-empty after strip
