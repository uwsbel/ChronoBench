import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens

sys = chrono.ChSystemNSC()                                            # rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no gravity; the body is fixed
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system (box body)

# --- box object visualized as a fixed body ---
side = 4.0                                                           # box edge length (m)
box = chrono.ChBodyEasyBox(side, side, side, 1000)                  # box, density 1000
box.SetPos(chrono.ChVector3d(0, 0, 0))                              # box position
box.SetFixed(True)                                                  # fixed in the scene
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # box texture
sys.Add(box)

# --- sensor manager (created before adding any sensor) ---
manager = sens.ChSensorManager(sys)

# --- lidar parameters shared by both sensors ---
update_rate = 5.0                                                   # lidar update rate (Hz)
horizontal_samples = 800                                            # horizontal beams
vertical_samples = 300                                              # vertical beams (3D lidar)
horizontal_fov = 2 * chrono.CH_PI                                   # 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                  # upper vertical FOV
min_vert_angle = -chrono.CH_PI / 6                                  # lower vertical FOV
max_lidar_range = 100.0                                             # max range (m)
sample_radius = 2                                                   # beam super-sample radius
divergence_angle = 0.003                                           # beam divergence (rad)

orbit_radius = 10.0                                                 # lidar orbit radius around the box
orbit_rate = 0.1                                                    # angular orbit rate (rad/s)

offset_pose = chrono.ChFramed(                                      # initial pose of the lidar on the box
    chrono.ChVector3d(-12, 0, 1),
    chrono.QUNIT,
)

# --- 3D lidar sensor attached to the box ---
lidar = sens.ChLidarSensor(
    box,                                                           # body the lidar rides on
    update_rate,                                                   # update rate (Hz)
    offset_pose,                                                   # offset pose on the body
    horizontal_samples,                                            # horizontal samples
    vertical_samples,                                              # vertical samples
    horizontal_fov,                                                # horizontal FOV (rad)
    max_vert_angle,                                                # max vertical angle (rad)
    min_vert_angle,                                                # min vertical angle (rad)
    max_lidar_range,                                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                               # beam shape
    sample_radius,                                                 # sample radius
    divergence_angle,                                              # vertical divergence
    divergence_angle,                                              # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                         # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)                                                    # lag = 0
lidar.SetCollectionWindow(1.0 / update_rate)                       # lidar collection window = 1 / rate

lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar/"))           # save the 3D lidar point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                        # host access to XYZI
manager.AddSensor(lidar)                                           # register the 3D lidar

# --- 2D lidar sensor (single vertical channel) attached to the box ---
horizontal_samples_2d = 800                                        # horizontal beams (2D)
lidar_2d = sens.ChLidarSensor(
    box,                                                           # body the 2D lidar rides on
    update_rate,                                                   # update rate (Hz)
    offset_pose,                                                   # offset pose on the body
    horizontal_samples_2d,                                         # horizontal samples
    1,                                                             # vertical samples = 1 (2D scan)
    horizontal_fov,                                                # horizontal FOV (rad)
    0.0,                                                           # max vertical angle = 0 (2D)
    0.0,                                                           # min vertical angle = 0 (2D)
    max_lidar_range,                                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                               # beam shape
    sample_radius,                                                 # sample radius
    divergence_angle,                                              # vertical divergence
    divergence_angle,                                              # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                         # return mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)                                                 # lag = 0
lidar_2d.SetCollectionWindow(1.0 / update_rate)                    # collection window = 1 / rate

lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, 1, "Raw 2D Lidar Depth Data"))  # depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())                       # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # point-cloud preview
lidar_2d.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_2d/"))     # save the 2D lidar point cloud
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                     # host access to XYZI
manager.AddSensor(lidar_2d)                                        # register the 2D lidar

# --- simulation loop (headless, time-bounded) ---
step_size = 1e-3                                                    # physics step
time_end = 40.0                                                     # scored end time
ch_time = 0.0                                                       # simulation clock


while ch_time < time_end:
    ch_time = sys.GetChTime()                                      # current sim time

    # orbit both lidars around the box by updating their offset pose each step
    orbit_pose = chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                          -orbit_radius * math.sin(ch_time * orbit_rate),
                          1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1)),
    )
    lidar.SetOffsetPose(orbit_pose)
    lidar_2d.SetOffsetPose(orbit_pose)

    manager.Update()                                               # pump both lidars once per step

    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()                  # read the latest 3D XYZI point cloud
    if xyzi_buffer.HasData():                                      # only after the first lidar tick
        xyzi = xyzi_buffer.GetXYZIData()
        print("XYZI buffer received. Resolution: {0}x{1}".format(xyzi_buffer.Width, xyzi_buffer.Height))
        print("Max XYZI value: ", np.max(xyzi))

    xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()            # read the latest 2D XYZI point cloud
    if xyzi_buffer_2d.HasData():                                   # only after the first 2D lidar tick
        xyzi_2d = xyzi_buffer_2d.GetXYZIData()
        print("2D XYZI buffer received. Resolution: {0}x{1}".format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
        print("Max 2D XYZI value: ", np.max(xyzi_2d))

    sys.DoStepDynamics(step_size)                                  # advance physics
