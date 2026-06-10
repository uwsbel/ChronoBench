import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens

sys = chrono.ChSystemNSC()                                            # rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no gravity; the body is fixed
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system (mesh body)

# --- triangular mesh from a Wavefront .obj, visualized as a fixed body ---
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))    # uniform scale x2

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual triangle mesh
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)                                       # static geometry

mesh_body = chrono.ChBody()                                           # the fixed body carrying the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape, chrono.ChFramed())
mesh_body.SetFixed(True)                                              # fixed in the scene
sys.Add(mesh_body)

# --- sensor manager (created before adding any sensor) ---
manager = sens.ChSensorManager(sys)

# --- lidar sensor attached to the mesh body ---
update_rate = 5.0                                                    # lidar update rate (Hz)
horizontal_samples = 800                                             # horizontal beams
vertical_samples = 300                                               # vertical beams
horizontal_fov = 2 * chrono.CH_PI                                    # 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                   # upper vertical FOV
min_vert_angle = -chrono.CH_PI / 6                                   # lower vertical FOV
max_lidar_range = 100.0                                              # max range (m)
sample_radius = 2                                                    # beam super-sample radius
divergence_angle = 0.003                                            # beam divergence (rad)

orbit_radius = 10.0                                                  # lidar orbit radius around the mesh
orbit_rate = 0.1                                                     # angular orbit rate (rad/s)

offset_pose = chrono.ChFramed(                                       # initial pose of the lidar on the body
    chrono.ChVector3d(-12, 0, 1),
    chrono.QUNIT,
)
lidar = sens.ChLidarSensor(
    mesh_body,                                                       # body the lidar rides on
    update_rate,                                                     # update rate (Hz)
    offset_pose,                                                     # offset pose on the body
    horizontal_samples,                                              # horizontal samples
    vertical_samples,                                                # vertical samples
    horizontal_fov,                                                  # horizontal FOV (rad)
    max_vert_angle,                                                  # max vertical angle (rad)
    min_vert_angle,                                                  # min vertical angle (rad)
    max_lidar_range,                                                 # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                                 # beam shape
    sample_radius,                                                   # sample radius
    divergence_angle,                                                # vertical divergence
    divergence_angle,                                                # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                           # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)                                                      # lag = 0
lidar.SetCollectionWindow(1.0 / update_rate)                         # lidar collection window = 1 / rate

# --- lidar filter chain (ORDER MATTERS) ---
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                        # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar/"))            # save the lidar point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                         # host access to XYZI
manager.AddSensor(lidar)                                            # register the lidar

# --- simulation loop (headless, time-bounded) ---
step_size = 1e-3                                                     # physics step
time_end = 40.0                                                      # scored end time
ch_time = 0.0                                                        # simulation clock


while ch_time < time_end:
    ch_time = sys.GetChTime()                                       # current sim time

    # orbit the lidar around the mesh by updating its offset pose each step
    lidar.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                          -orbit_radius * math.sin(ch_time * orbit_rate),
                          1),
        chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1)),
    ))

    manager.Update()                                                # pump the lidar once per step

    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()                   # read the latest XYZI point cloud
    if xyzi_buffer.HasData():                                       # only after the first lidar tick
        xyzi = xyzi_buffer.GetXYZIData()
        print("XYZI buffer received. Resolution: {0}x{1}".format(xyzi_buffer.Width, xyzi_buffer.Height))
        print("Max XYZI value: ", np.max(xyzi))

    sys.DoStepDynamics(step_size)                                   # advance physics
