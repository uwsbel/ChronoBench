import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

import os
import numpy as np
import time
import math


def main():
    # create the physical system
    mphysicalSystem = chrono.ChSystemNSC()

    # load a wavefront obj mesh from the chrono data path
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))  # scale mesh by factor 2

    # create the visual shape wrapping the mesh
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    # create a fixed body to hold the mesh
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    # create the sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # lidar parameters
    update_rate = 5.0                            # scanning rate in Hz
    horizontal_samples = 800                     # horizontal samples per scan
    vertical_samples = 300                       # vertical channels
    horizontal_fov = 2 * chrono.CH_PI            # 360-degree horizontal fov
    max_vert_angle = chrono.CH_PI / 12           # max vertical angle
    min_vert_angle = -chrono.CH_PI / 6           # min vertical angle
    sample_radius = 2                            # sample radius
    divergence_angle = 0.003                     # beam divergence (rad)
    lag = 0                                      # sensor lag
    collection_time = 1.0 / update_rate          # collection window = 1/rate
    noise_model = "NONE"                         # no noise model for this demo

    # initial offset pose for the lidar (at -12, 0, 1)
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    # create the lidar sensor attached to the fixed mesh body
    lidar = sens.ChLidarSensor(
        mesh_body,                               # body lidar is attached to
        update_rate,                             # scanning rate in Hz
        offset_pose,                             # offset pose
        horizontal_samples,                      # horizontal samples per scan
        vertical_samples,                        # vertical channels
        horizontal_fov,                          # horizontal field of view (rad)
        max_vert_angle,                          # max vertical angle (rad)
        min_vert_angle,                          # min vertical angle (rad)
        100.0,                                   # max lidar range
        sens.LidarBeamShape_RECTANGULAR,         # beam shape
        sample_radius,                           # sample radius
        divergence_angle,                        # vertical divergence angle
        divergence_angle,                        # horizontal divergence angle
        sens.LidarReturnMode_STRONGEST_RETURN,   # return mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # apply noise filter if requested
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    # visualize raw lidar depth data
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # host access to depth+intensity buffer
    lidar.PushFilter(sens.ChFilterDIAccess())

    # convert depth+intensity to XYZI point cloud
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    # visualize the XYZI point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # host access to XYZI point cloud data (scored core — untagged)
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # save point cloud data to disk (scored core output — untagged)
    lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_pc/"))

    # add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # build the Irrlicht visualization window (unconditionally built)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(mphysicalSystem)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Lidar Sensor Demo")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-15, 0, 5), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # simulation parameters
    step_size = 1e-3                             # physics step size
    end_time = 40.0                              # simulation end time (truth: 40s)
    orbit_radius = 10                            # lidar orbit radius around mesh
    orbit_rate = 0.1                             # lidar orbit angular rate (rad/s)

    render_fps = 30.0                            # Irrlicht rendering frame rate
    render_every = max(1, round(1.0 / (render_fps * step_size)))  # render cadence

    t1 = time.time()

    while vis.Run() and mphysicalSystem.GetChTime() < end_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            ch_time = mphysicalSystem.GetChTime()
            # orbit the lidar dynamically around the mesh body
            lidar.SetOffsetPose(
                chrono.ChFramed(
                    chrono.ChVector3d(
                        -orbit_radius * math.cos(ch_time * orbit_rate),
                        -orbit_radius * math.sin(ch_time * orbit_rate),
                        1
                    ),
                    chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
                )
            )
            # access and print the XYZI buffer from the lidar (scored core)
            xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
            if xyzi_buffer.HasData():
                xyzi_data = xyzi_buffer.GetXYZIData()
                print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
                print('Max Value: {0}'.format(np.max(xyzi_data)))
            # update sensor manager (renders/saves/filters all sensors)
            manager.Update()
            # advance physics one step
            mphysicalSystem.DoStepDynamics(step_size)
            if mphysicalSystem.GetChTime() >= end_time:
                break

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()
