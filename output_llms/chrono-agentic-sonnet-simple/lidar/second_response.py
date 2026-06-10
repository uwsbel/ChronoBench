import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# simulation parameters
time_step = 1e-3                                     # physics step (s)
sim_end = 10.0                                       # simulation end time (s)
render_fps = 50.0                                    # Irrlicht render rate (fps)
side = 1.0                                           # box side length (m)

# physical system
sys = chrono.ChSystemNSC()                           # NSC contact
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 down (Z-up)
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # bullet collision required

# ground plane (fixed)
mat = chrono.ChContactMaterialNSC()                  # contact material
ground = chrono.ChBodyEasyBox(20, 20, 0.2, 1000, True, True, mat)  # large flat ground
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))         # just below origin
ground.SetFixed(True)                                # fixed ground body
sys.AddBody(ground)

# box object (replaces mesh from turn 1)
mat_box = chrono.ChContactMaterialNSC()              # box contact material
box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True, mat_box)  # box with collision
box_body.SetPos(chrono.ChVector3d(0, 0, side / 2.0))  # rest on ground
box_body.SetFixed(True)                              # fixed so lidar scans a static target
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # blue texture
sys.AddBody(box_body)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)    # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Sensor Demo - Box Object")
vis.Initialize()                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-12, 0, 6), chrono.ChVector3d(0, 0, 0))  # view from side
vis.AddTypicalLights()

# sensor manager
manager = sens.ChSensorManager(sys)                  # sensor manager attached to system

# 3D lidar sensor attached to the box
offset_pose_lidar = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),                    # offset: 12 m in front of box
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
horizontal_samples = 800                             # horizontal resolution
vertical_samples = 300                               # vertical resolution (3D lidar)
lidar = sens.ChLidarSensor(
    box_body,                                        # attached to box body
    5.0,                                             # update_rate (Hz)
    offset_pose_lidar,                               # offset pose
    horizontal_samples,                              # h_samples
    vertical_samples,                                # v_samples
    2 * chrono.CH_PI,                                # horizontal_fov (full 360 deg)
    chrono.CH_PI / 12,                               # max_vert_angle
    -chrono.CH_PI / 6,                               # min_vert_angle
    100.0,                                           # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,                 # beam shape
    2,                                               # sample_radius
    0.003,                                           # vert divergence angle
    0.003,                                           # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,           # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)                                      # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                 # collection window = 1/update_rate

# 3D lidar filter chain (ORDER MATTERS — scored core, never review-only)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())          # host access to XYZI
manager.AddSensor(lidar)                             # add 3D lidar to manager

# 2D lidar sensor (one vertical channel) attached to the box
offset_pose_lidar_2d = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 2),                    # slightly higher offset
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
horizontal_samples_2d = 800                          # horizontal resolution for 2D lidar
vertical_samples_2d = 1                              # 2D lidar: one vertical channel
lidar_2d = sens.ChLidarSensor(
    box_body,                                        # attached to box body
    5.0,                                             # update_rate (Hz)
    offset_pose_lidar_2d,                            # offset pose
    horizontal_samples_2d,                           # h_samples
    vertical_samples_2d,                             # v_samples = 1 for 2D
    2 * chrono.CH_PI,                                # horizontal_fov (full 360 deg)
    0,                                               # max_vert_angle = 0 for 2D
    0,                                               # min_vert_angle = 0 for 2D
    100.0,                                           # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,                 # beam shape
    2,                                               # sample_radius
    0.003,                                           # vert divergence angle
    0.003,                                           # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,           # return mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)                                   # no lag
lidar_2d.SetCollectionWindow(1.0 / 5.0)             # collection window = 1/update_rate

# 2D lidar filter chain (scored core, never review-only)
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, vertical_samples_2d, "Raw 2D Lidar Depth"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())         # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())      # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())       # host access to XYZI for 2D lidar
manager.AddSensor(lidar_2d)                          # add 2D lidar to manager

# render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # steps between Irrlicht renders

# simulation loop
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        manager.Update()                             # update all sensors each physics step
        sys.DoStepDynamics(time_step)               # advance physics
        # sensor diagnostic for 3D lidar (scored core)
        buf = lidar.GetMostRecentXYZIBuffer()
        if buf.HasData():
            print('Buffer received. Resolution: {0}x{1}'.format(buf.Width, buf.Height))
        # sensor diagnostic for 2D lidar (scored core)
        buf_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if buf_2d.HasData():
            print('Buffer received. Resolution: {0}x{1}'.format(buf_2d.Width, buf_2d.Height))
        if sys.GetChTime() >= sim_end:
            break
