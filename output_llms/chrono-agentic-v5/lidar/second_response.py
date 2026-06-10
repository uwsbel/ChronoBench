"""Lidar sensing of a static box target with a 3D and a 2D lidar (PyChrono 9.0.0).

Models an NSC rigid-body scene whose single protagonist is a fixed cubic box
(side x side x side) sitting at the world origin. Two OptiX lidar sensors are
attached to that box and orbit/scan it:

  * a 3D rotating lidar with multiple vertical channels (32), full 360 deg
    horizontal FOV, producing a depth image and an XYZI point cloud, and
  * a 2D lidar with a single vertical channel (both vertical angles 0), a planar
    360 deg sweep.

Both lidars push the standard depth/intensity -> point-cloud filter chain with
live visualization. The box is static, so the expected behavior is a stable,
well-formed point cloud of the cube's faces from each lidar's moving viewpoint.
Irrlicht provides the review window; the lidars provide the sensor data.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Parameters === geometry, lidar config, and simulation timing constants
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0

side = 4.0                      # cube edge length (m)
box_density = 1000.0           # density for ChBodyEasyBox
box_pos = chrono.ChVector3d(0, 0, 0)

# Lidar scan configuration (physical rates, not 1/dt).
update_rate = 5.0                          # Hz
horizontal_samples = 4500                  # 3D lidar horizontal samples
vertical_samples = 32                      # 3D lidar vertical channels
horizontal_fov = 2 * chrono.CH_PI          # 360 deg sweep
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
max_range = 100.0
sample_radius = 2
divergence_angle = 0.003                   # 3 mm beam divergence (velodyne)
collection_time = 1.0 / update_rate        # lidar collection window = 1/rate

horizontal_samples_2d = 4500               # 2D lidar horizontal samples
vertical_samples_2d = 1                    # single channel -> planar 2D lidar

render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === NSC system; Bullet collision (box has a collision shape)
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === static cubic box target that the lidars sense
box_body = chrono.ChBodyEasyBox(side, side, side, box_density)
box_body.SetPos(box_pos)
box_body.SetFixed(True)
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(box_body)

# === Sensors === sensor manager + a 3D lidar and a 2D lidar attached to the box
manager = sens.ChSensorManager(sys)

# --- 3D lidar: 360 deg sweep with 32 vertical channels ---
offset_pose_3d = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    box_body,                          # body the lidar is attached to
    update_rate,                       # scanning rate (Hz)
    offset_pose_3d,                    # offset pose
    horizontal_samples,                # horizontal samples
    vertical_samples,                  # vertical channels
    horizontal_fov,                    # horizontal FOV (rad)
    max_vert_angle,                    # max vertical angle (rad)
    min_vert_angle,                    # min vertical angle (rad)
    max_range,                         # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,                     # sample radius
    divergence_angle,                  # vertical divergence
    divergence_angle,                  # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(collection_time)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
lidar.PushFilter(sens.ChFilterDIAccess())              # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())           # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())            # host access to XYZI
manager.AddSensor(lidar)

# --- 2D lidar: single vertical channel (planar sweep), both vert angles 0 ---
offset_pose_2d = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    box_body,                          # body the 2D lidar is attached to
    update_rate,                       # scanning rate (Hz)
    offset_pose_2d,                    # offset pose
    horizontal_samples_2d,             # horizontal samples
    vertical_samples_2d,               # single vertical channel
    horizontal_fov,                    # horizontal FOV (rad)
    0.0,                               # max vertical angle = 0 (planar)
    0.0,                               # min vertical angle = 0 (planar)
    max_range,                         # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,                     # sample radius
    divergence_angle,                  # vertical divergence
    divergence_angle,                  # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)
lidar_2d.SetCollectionWindow(collection_time)
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, vertical_samples_2d, "Raw 2D Lidar Depth Data"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar_2d)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar sensing of a box target")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-12, -12, 8), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -side / 2), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Lidar orbit pose === precompute the orbit so the moving viewpoint scans all faces
orbit_radius = 5.0      # precomputed once
orbit_rate = 0.2        # precomputed once

# === Main loop === render-cadence outer loop; advance physics + pump sensors inner

os.makedirs("cam", exist_ok=True)   # guard against missing output dir
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        t = sys.GetChTime()
        # Orbit both lidars around the box so the scan sweeps every face.
        orbit_pose = chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(t * orbit_rate),
                              -orbit_radius * math.sin(t * orbit_rate), 1),
            chrono.QuatFromAngleAxis(t * orbit_rate, chrono.ChVector3d(0, 0, 1)))
        lidar.SetOffsetPose(orbit_pose)
        lidar_2d.SetOffsetPose(orbit_pose)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                # pump both lidars every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
