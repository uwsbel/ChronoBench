import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    system = chrono.ChSystemNSC()

    # ---------------------------------------------------
    # Replace mesh with a simple box created by EasyBox
    # ---------------------------------------------------
    side_length = 2.0
    density = 1000.0
    # ChBodyEasyBox(dx, dy, dz, density, do_visualize=True, do_collide=False)
    box_body = chrono.ChBodyEasyBox(
        side_length, side_length, side_length, density, True, False
    )
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetBodyFixed(True)
    # Add a simple texture
    tex = chrono.ChTexture()
    tex.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    box_body.AddAsset(tex)

    system.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(system)

    # ------------------------------------------
    # 1) Create / configure the original 3D lidar
    # ------------------------------------------
    # offset pose relative to the box
    offset3D = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_3d = sens.ChLidarSensor(
        box_body,                 # attach to box
        update_rate,              # Hz
        offset3D,                 # pose
        horizontal_samples,       # H samples
        vertical_samples,         # V channels
        horizontal_fov,           # horizontal FOV
        max_vert_angle,           # max vertical
        min_vert_angle,           # min vertical
        max_range,                # max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_3d.SetName("Lidar3D")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # optionally add noise
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    # visualize raw depth
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, vertical_samples, "3D Lidar Depth"
        ))

    # host‐access filter + depth→point‐cloud + host‐access XYZI
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(
            640, 480, 1.0, "3D Point Cloud"
        ))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_3d)

    # ------------------------------------------
    # 2) Create / configure the additional 2D lidar
    # ------------------------------------------
    # We'll make it a single‐channel lidar, i.e. true "2D"
    offset2D = chrono.ChFrameD(
        chrono.ChVectorD(0, -5, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = sens.ChLidarSensor(
        box_body,                 # attach to same box
        update_rate,              # Hz
        offset2D,                 # pose
        horizontal_samples,       # H samples
        1,                        # single vertical channel
        horizontal_fov,           # full horizontal FOV
        0.0,                      # max vertical = 0
        0.0,                      # min vertical = 0
        max_range,                # same max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        0.0,                      # no divergence vertically
        0.0,
        return_mode
    )
    lidar_2d.SetName("Lidar2D")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, 1, "2D Lidar Depth"
        ))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # We do not build a full 3D point cloud for 2D lidar
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulation loop
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t_start = time.time()
    while ch_time < end_time:
        # orbit both lidars around the box
        angle = ch_time * orbit_rate
        pos = chrono.ChVector3d(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1
        )
        quat = chrono.ChQuaternionD(math.cos(angle/2), 0, 0, math.sin(angle/2))
        new_pose = chrono.ChFrameD(pos, quat)

        lidar_3d.SetOffsetPose(new_pose)
        lidar_2d.SetOffsetPose(new_pose)

        # pull data from 3D lidar
        buf3 = lidar_3d.GetMostRecentXYZIBuffer()
        if buf3.HasData():
            data3 = buf3.GetXYZIData()
            print(f"[3D] got buffer {buf3.Width}×{buf3.Height}, max={np.max(data3)}")

        # pull data from 2D lidar
        buf2 = lidar_2d.GetMostRecentDepthBuffer()
        if buf2.HasData():
            d2 = buf2.GetDepthData()
            print(f"[2D] got depth array length {buf2.Width}, max={np.max(d2)}")

        # advance sensors and dynamics
        manager.Update()
        system.DoStepDynamics(step_size)
        ch_time = system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t_start)


# -----------------
# Lidar parameters
# -----------------
noise_model = "NONE"                 # or "CONST_NORMAL_XYZI"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
max_range = 100.0
sample_radius = 2
divergence_angle = 0.003
lag = 0
collection_time = 1.0 / update_rate

# ----------------------
# Simulation parameters
# ----------------------
step_size = 1e-3
end_time = 40.0
vis = True

if __name__ == "__main__":
    # If you need to override the default data path:
    # chrono.SetChronoDataPath("/your/chrono/data/")
    main()