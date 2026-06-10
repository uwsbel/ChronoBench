import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))      # g = 9.81 down
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system (sensor renders collision geom)

side = 2.0                                                            # cube edge length (m)
box_body = chrono.ChBodyEasyBox(side, side, side, 1000)              # box replaces the triangle mesh
box_body.SetPos(chrono.ChVector3d(0, 0, 0))                          # box at the world origin
box_body.SetFixed(True)                                              # the sensed object stays put
box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # surface texture
sys.Add(box_body)                                                   # add the box to the system

ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, False)         # ground plane (no contact)
ground.SetPos(chrono.ChVector3d(0, 0, -1))                          # below the box
ground.SetFixed(True)                                               # static ground
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # ground texture
sys.Add(ground)                                                    # add ground

manager = sens.ChSensorManager(sys)                                # sensor manager oversees all sensors
manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100),      # key point light
                            chrono.ChColor(1, 1, 1), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-100, -100, 100),    # fill point light
                            chrono.ChColor(1, 1, 1), 5000.0)

update_rate = 5.0                                                  # lidar update rate (Hz)
horizontal_samples = 800                                          # horizontal beam count
vertical_samples = 300                                            # vertical beam count (3D lidar)
max_vert_angle = chrono.CH_PI / 12                                # +15 deg up
min_vert_angle = -chrono.CH_PI / 6                                # -30 deg down
horizontal_fov = 2 * chrono.CH_PI                                 # full 360 deg sweep
max_distance = 100.0                                              # max range (m)
lag = 0.0                                                         # acquisition lag
collection_time = 1.0 / update_rate                              # collection window = 1 / update_rate
sample_radius = 2                                                 # super-sample radius
divergence_angle = 0.003                                         # beam divergence (rad)
offset_radius = 8.0                                              # lidar orbit radius around the box

offset_pose = chrono.ChFramed(                                   # 3D lidar pose relative to the box
    chrono.ChVector3d(-offset_radius, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    box_body,                                                   # attach the lidar to the box
    update_rate,                                                # update rate (Hz)
    offset_pose,                                                # offset pose
    horizontal_samples,                                         # horizontal samples
    vertical_samples,                                           # vertical samples (3D)
    horizontal_fov,                                             # horizontal fov (rad)
    max_vert_angle,                                             # max vertical angle
    min_vert_angle,                                             # min vertical angle
    max_distance,                                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                            # rectangular beam
    sample_radius,                                              # sample radius
    divergence_angle,                                           # vertical divergence
    divergence_angle,                                           # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN)                      # strongest return
lidar.SetName("Lidar Sensor")                                  # name the 3D lidar
lidar.SetLag(lag)                                              # acquisition lag
lidar.SetCollectionWindow(collection_time)                    # collection window
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))  # raw depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                     # access depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                  # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                   # access the XYZI point cloud
manager.AddSensor(lidar)                                      # register the 3D lidar

offset_pose_2d = chrono.ChFramed(                            # 2D lidar pose relative to the box
    chrono.ChVector3d(-offset_radius, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar_2d = sens.ChLidarSensor(
    box_body,                                               # attach the 2D lidar to the box
    update_rate,                                            # update rate (Hz)
    offset_pose_2d,                                         # offset pose
    horizontal_samples,                                    # horizontal samples
    1,                                                     # single vertical channel -> 2D lidar
    horizontal_fov,                                        # horizontal fov (rad)
    0.0,                                                   # max vertical angle = 0 (planar)
    0.0,                                                   # min vertical angle = 0 (planar)
    max_distance,                                          # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                       # rectangular beam
    sample_radius,                                         # sample radius
    divergence_angle,                                      # vertical divergence
    divergence_angle,                                      # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN)                 # strongest return
lidar_2d.SetName("2D Lidar Sensor")                       # name the 2D lidar
lidar_2d.SetLag(lag)                                      # acquisition lag
lidar_2d.SetCollectionWindow(collection_time)            # collection window
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))  # raw 2D depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())             # access depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())          # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # 2D point-cloud preview
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())           # access the XYZI point cloud
manager.AddSensor(lidar_2d)                              # register the 2D lidar

vis = chronoirr.ChVisualSystemIrrlicht()                 # Irrlicht review window
vis.AttachSystem(sys)                                    # attach the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)        # Z-up camera
vis.SetWindowSize(1280, 720)                             # window resolution
vis.SetWindowTitle("Lidar Sensor Demo")                 # window title
vis.Initialize()                                         # init device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                          # sky box
vis.AddCamera(chrono.ChVector3d(-12, -12, 8), chrono.ChVector3d(0, 0, 0))  # eye, target
vis.AddTypicalLights()                                   # standard lights

time_step = 1e-3                                         # integration step (s)
sim_end = 10.0                                           # simulation end time (s)
orbit_rate = chrono.CH_PI / 6                            # lidar orbit angular rate (rad/s)
render_fps = 30.0                                        # review render cadence (Hz)
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
ch_time = 0.0                                            # simulation clock

while vis.Run() and ch_time < sim_end:
    vis.BeginScene()                                    # begin frame
    vis.Render()                                        # draw scene
    vis.EndScene()                                      # end frame
    for _ in range(render_every):
        ch_time = sys.GetChTime()                       # current sim time
        # orbit the lidars around the box so the sweep covers the cube faces
        new_pose = chrono.ChFramed(
            chrono.ChVector3d(-offset_radius * math.cos(orbit_rate * ch_time),
                              -offset_radius * math.sin(orbit_rate * ch_time),
                              1),
            chrono.QuatFromAngleAxis(orbit_rate * ch_time, chrono.ChVector3d(0, 0, 1)))
        lidar.SetOffsetPose(new_pose)                   # move the 3D lidar
        lidar_2d.SetOffsetPose(new_pose)                # move the 2D lidar
        manager.Update()                                # pump sensors once per step
        sys.DoStepDynamics(time_step)                   # advance physics
        # read the 3D lidar point cloud when a new buffer is ready
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi = xyzi_buffer.GetXYZIData()
            print("3D Lidar buffer received. Points: ", xyzi_buffer.Width * xyzi_buffer.Height)
        # read the 2D lidar point cloud when a new buffer is ready
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            print("2D Lidar buffer received. Beams: ", xyzi_buffer_2d.Width * xyzi_buffer_2d.Height)
        if sys.GetChTime() >= sim_end:
            break
