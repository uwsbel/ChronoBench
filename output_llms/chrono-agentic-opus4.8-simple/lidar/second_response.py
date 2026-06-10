import os
import pychrono.core as chrono                                       # core multibody
import pychrono.sensor as sens                                        # sensor (OptiX) module
import pychrono.irrlicht as chronoirr                                 # Irrlicht review window

sys = chrono.ChSystemNSC()                                           # NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # g = 9.81 down
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # bullet collision for the box

side = 4.0                                                           # box edge length (m)
box = chrono.ChBodyEasyBox(side, side, side, 1000)                   # box object, density 1000
box.SetPos(chrono.ChVector3d(0, 0, 0))                              # place box at the origin
box.SetFixed(True)                                                   # keep the target static for the scan
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # box texture
sys.Add(box)                                                         # add box to the system

manager = sens.ChSensorManager(sys)                                 # sensor manager oversees the lidars

update_rate = 5.0                                                   # lidar physical update rate (Hz)
horizontal_samples = 800                                             # horizontal beam samples
vertical_samples = 300                                               # vertical beam samples (3D lidar)

offset_pose = chrono.ChFramed(                                      # 3D lidar offset pose on the box
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    box,                                                            # attach the lidar to the box
    update_rate,                                                    # update rate (Hz)
    offset_pose,                                                    # offset pose
    horizontal_samples,                                            # h_samples
    vertical_samples,                                              # v_samples
    2 * chrono.CH_PI,                                             # horizontal fov (rad)
    chrono.CH_PI / 12,                                           # max vertical angle
    -chrono.CH_PI / 6,                                           # min vertical angle
    100.0,                                                         # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                              # beam shape
    2,                                                            # sample radius
    0.003,                                                        # vertical divergence angle
    0.003,                                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                        # return mode
)
lidar.SetName("Lidar Sensor")                                      # 3D lidar name
lidar.SetLag(0)                                                    # no lag
lidar.SetCollectionWindow(1.0 / update_rate)                      # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                         # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                     # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                       # host access to XYZI
manager.AddSensor(lidar)                                           # register the 3D lidar

vertical_samples_2d = 1                                            # 2D lidar has one vertical channel
offset_pose_2d = chrono.ChFramed(                                  # 2D lidar offset pose on the box
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar_2d = sens.ChLidarSensor(
    box,                                                            # attach the 2D lidar to the box
    update_rate,                                                    # update rate (Hz)
    offset_pose_2d,                                                # offset pose
    horizontal_samples,                                           # h_samples
    vertical_samples_2d,                                          # v_samples = 1 for 2D
    2 * chrono.CH_PI,                                             # horizontal fov (rad)
    0.0,                                                          # max vertical angle = 0 (planar)
    0.0,                                                          # min vertical angle = 0 (planar)
    100.0,                                                         # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                              # beam shape
    2,                                                            # sample radius
    0.003,                                                        # vertical divergence angle
    0.003,                                                        # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                        # return mode
)
lidar_2d.SetName("2D Lidar Sensor")                               # 2D lidar name
lidar_2d.SetLag(0)                                                # no lag
lidar_2d.SetCollectionWindow(1.0 / update_rate)                  # collection window = 1 / update_rate
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples_2d, "Raw 2D Lidar Depth"))  # depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())                     # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                 # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # point-cloud preview
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                   # host access to XYZI
manager.AddSensor(lidar_2d)                                       # register the 2D lidar

vis = chronoirr.ChVisualSystemIrrlicht()                         # Irrlicht review window
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)               # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Box Scan")
vis.Initialize()                                                 # initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-16, -8, 6), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()

time_step = 1e-3                                                 # physics step
sim_end = 10.0                                                   # simulation end time (s)
render_fps = 50.0                                               # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        manager.Update()                                       # pump both lidars once per physics step
        sys.DoStepDynamics(time_step)                          # advance one physics step
        di_buffer = lidar.GetMostRecentDIBuffer()             # most recent depth+intensity buffer
        if di_buffer.HasData():                                # only read after the lidar has filled it
            print('3D lidar buffer received. Points: {0}x{1}'.format(di_buffer.Width, di_buffer.Height))
        di_buffer_2d = lidar_2d.GetMostRecentDIBuffer()       # most recent 2D depth+intensity buffer
        if di_buffer_2d.HasData():                             # only read after the 2D lidar has filled it
            print('2D lidar buffer received. Points: {0}x{1}'.format(di_buffer_2d.Width, di_buffer_2d.Height))
        if sys.GetChTime() >= sim_end:
            break
