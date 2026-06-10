"""Lidar sensing of a static Wavefront mesh.

Models a single fixed triangular mesh body (loaded from an .obj file) sensed by a
ChLidarSensor managed by a ChSensorManager. System type is NSC; the only body is a
fixed, non-colliding visual mesh, so no gravity-driven motion occurs. The lidar runs
a filter graph (optional noise -> raw-depth visualize -> depth/intensity access ->
point-cloud conversion -> point-cloud visualize -> XYZI access), and its offset pose
is updated every step so the sensor orbits the mesh on a circle while always facing
inward. The XYZI buffer is read each step and its data printed. Expected behavior: the
lidar buffer fills after the first scan and the XYZI point cloud sweeps around the
stationary mesh for the whole run.
"""

import math

import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np


# === Parameters === lidar + sim constants (direct values, matching the demo truth)
noise_model = "NONE"          # lidar noise model branch ("NONE" or "CONST_NORMAL_XYZI")
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0             # lidar scan rate (Hz) — physical rate, not 1/dt
horizontal_samples = 4500     # horizontal beam count
vertical_samples = 32         # vertical channel count
horizontal_fov = 2 * chrono.CH_PI          # 360 deg horizontal sweep
max_vert_angle = chrono.CH_PI / 12         # upper vertical bound (rad)
min_vert_angle = -chrono.CH_PI / 6         # lower vertical bound (rad)
max_range = 100.0             # max lidar range (m)
sample_radius = 2             # multisampling radius
divergence_angle = 0.003      # beam divergence (rad, ~3 mm as cited by Velodyne)
lag = 0                       # sensor lag
collection_time = 1.0 / update_rate        # collection window = 1 / update_rate

step_size = 1e-3              # dynamics step
end_time = 20.0               # simulation end time
orbit_radius = 5.0            # lidar orbit radius about the mesh
orbit_rate = 0.2             # orbit angular rate (rad/s of the cos/sin argument)

# === System & gravity === NSC rigid system; the sensed mesh is fixed (no dynamics)
mphysicalSystem = chrono.ChSystemNSC()

# === Bodies === load the Wavefront .obj, scale it, attach as a fixed visual mesh
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))   # scale up 2x

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)
mphysicalSystem.Add(mesh_body)

# === Sensor manager === oversees the lidar and renders its filter graph
manager = sens.ChSensorManager(mphysicalSystem)

# === Lidar sensor === 3D lidar attached to the mesh body with an orbiting offset pose
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    mesh_body,                 # body the lidar is attached to
    update_rate,               # scanning rate (Hz)
    offset_pose,               # offset pose on the body
    horizontal_samples,        # horizontal samples
    vertical_samples,          # vertical channels
    horizontal_fov,            # horizontal field of view (rad)
    max_vert_angle,            # max vertical angle (rad)
    min_vert_angle,            # min vertical angle (rad)
    max_range,                 # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,             # sample radius
    divergence_angle,          # vertical divergence angle
    divergence_angle,          # horizontal divergence angle
    return_mode,               # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# === Lidar filter graph === noise -> visualize -> access -> point cloud -> visualize/access
if noise_model == "CONST_NORMAL_XYZI":
    # Gaussian XYZI noise on the returns (constant mean/stdev)
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
elif noise_model == "NONE":
    pass                       # no noise applied to the lidar returns

lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to the XYZI buffer
manager.AddSensor(lidar)

# === Main loop === orbit the lidar around the mesh, pump sensors, save/print buffer data


try:
    ch_time = 0.0
    while ch_time < end_time:
        # Drive the offset pose so the lidar orbits the mesh, facing inward.
        lidar_x = -orbit_radius * math.cos(ch_time * orbit_rate)
        lidar_y = -orbit_radius * math.sin(ch_time * orbit_rate)
        lidar.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(lidar_x, lidar_y, 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        # Access the XYZI buffer; guard: empty until the lidar's first scan completes.
        max_val = 0.0
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            max_val = float(np.max(xyzi_data))
            print('XYZI buffer recieved from lidar. Lidar resolution: {0}x{1}'
                  .format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(max_val))


        manager.Update()                       # render/filter the lidar each step
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()
except (RuntimeError, ValueError) as exc:      # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
