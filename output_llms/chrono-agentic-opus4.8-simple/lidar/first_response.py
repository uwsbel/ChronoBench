import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

sys = chrono.ChSystemNSC()                                            # rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system (mesh body)

mmesh = chrono.ChTriangleMeshConnected()                              # triangular mesh container
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load .obj
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))    # scale the mesh up 2x

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual shape from the mesh
trimesh_shape.SetMesh(mmesh)                                          # bind the loaded mesh
trimesh_shape.SetName("Mesh")                                         # shape name
trimesh_shape.SetMutable(False)                                       # static geometry

mesh_body = chrono.ChBody()                                           # body carrying the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                          # at the origin
mesh_body.AddVisualShape(trimesh_shape)                               # attach the visual mesh
mesh_body.SetFixed(True)                                              # fixed in the scene
sys.Add(mesh_body)                                                    # add to the system

manager = sens.ChSensorManager(sys)                                   # sensor manager oversees the lidar

update_rate = 5.0                                                     # lidar scan rate (Hz)
horizontal_samples = 4500                                            # horizontal samples
vertical_samples = 32                                                # vertical channels
horizontal_fov = 2 * chrono.CH_PI                                    # 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                   # upper vertical angle
min_vert_angle = -chrono.CH_PI / 6                                   # lower vertical angle
sample_radius = 2                                                    # beam sample radius
divergence_angle = 0.003                                             # 3 mm beam divergence

offset_pose = chrono.ChFramed(                                        # initial lidar pose on the body
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    mesh_body,                                                       # body the lidar is attached to
    update_rate,                                                     # update rate (Hz)
    offset_pose,                                                     # offset pose
    horizontal_samples,                                             # h samples
    vertical_samples,                                               # v samples
    horizontal_fov,                                                 # horizontal FOV (rad)
    max_vert_angle,                                                 # max vertical angle
    min_vert_angle,                                                 # min vertical angle
    100.0,                                                          # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                                # beam shape
    sample_radius,                                                  # sample radius
    divergence_angle,                                               # vertical divergence
    divergence_angle,                                               # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                          # return mode
)
lidar.SetName("Lidar Sensor")                                        # sensor name
lidar.SetLag(0)                                                     # no lag
lidar.SetCollectionWindow(1.0 / update_rate)                        # collection window = 1/update_rate

lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterSave("lidar/depth/"))                 # save the raw depth/intensity frames
lidar.PushFilter(sens.ChFilterDIAccess())                           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                        # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                         # host access to XYZI
manager.AddSensor(lidar)                                            # register the lidar

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht window
vis.AttachSystem(sys)                                               # attach the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up
vis.SetWindowSize(1280, 720)                                        # window size
vis.SetWindowTitle("Lidar Sensor")                                 # window title
vis.Initialize()                                                   # init first, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # window camera
vis.AddTypicalLights()                                            # standard lights

time_step = 1e-3                                                   # physics step
sim_end = 20.0                                                     # end time
render_fps = 50.0                                                  # render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
orbit_radius = 5                                                   # lidar orbit radius
orbit_rate = 0.2                                                   # lidar orbit angular rate


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        t = sys.GetChTime()                                       # current sim time
        lidar.SetOffsetPose(chrono.ChFramed(                      # orbit the lidar around the mesh
            chrono.ChVector3d(-orbit_radius * math.cos(t * orbit_rate),
                              -orbit_radius * math.sin(t * orbit_rate), 1),
            chrono.QuatFromAngleAxis(t * orbit_rate, chrono.ChVector3d(0, 0, 1))))
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()             # most recent point cloud
        if xyzi_buffer.HasData():                                 # only after the first lidar tick
            xyzi_data = xyzi_buffer.GetXYZIData()                 # host-side XYZI array
            print('XYZI buffer recieved from lidar. Lidar resolution: {0}x{1}'
                  .format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))
        manager.Update()                                         # pump the lidar once per step
        sys.DoStepDynamics(time_step)                            # advance dynamics
        if sys.GetChTime() >= sim_end:
            break
