import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

mphysicalSystem = chrono.ChSystemNSC()                                 # NSC system for the static mesh scene

# Load the triangular mesh from a Wavefront .obj file
mmesh = chrono.ChTriangleMeshConnected()                              # triangle mesh container
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load + compute normals
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))    # scale the mesh up by 2x

trimesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual wrapper for the mesh
trimesh_shape.SetMesh(mmesh)                                          # bind the loaded mesh
trimesh_shape.SetName("HMMWV Chassis Mesh")                          # name the visual asset
trimesh_shape.SetMutable(False)                                      # static geometry — never re-uploaded

mesh_body = chrono.ChBody()                                          # body that carries the mesh
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                         # placed at the world origin
mesh_body.AddVisualShape(trimesh_shape)                             # attach the triangle-mesh visual
mesh_body.SetFixed(True)                                            # fixed body — the scene is static
mphysicalSystem.Add(mesh_body)                                      # add the mesh body to the system

# Create the sensor manager that drives the lidar
manager = sens.ChSensorManager(mphysicalSystem)                     # manages all sensors / OptiX rendering

# Lidar parameters (Velodyne-like 360 deg scanner)
update_rate = 5.0                                                    # scanning rate in Hz
horizontal_samples = 4500                                           # horizontal samples per revolution
vertical_samples = 32                                               # vertical channels
horizontal_fov = 2 * chrono.CH_PI                                   # 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                  # upper vertical bound
min_vert_angle = -chrono.CH_PI / 6                                  # lower vertical bound
sample_radius = 2                                                    # super-sampling radius (2 -> 9 samples)
divergence_angle = 0.003                                            # 3 mm beam divergence (Velodyne)
lag = 0                                                             # no sensing lag
collection_time = 1.0 / update_rate                                 # collection window = 1 / update rate
return_mode = sens.LidarReturnMode_STRONGEST_RETURN                 # strongest-return mode
noise_model = "NONE"                                               # noise model selector (NONE / CONST_NORMAL_XYZI)

offset_pose = chrono.ChFramed(                                       # initial pose of the lidar on the body
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    mesh_body,                                                      # body the lidar is attached to
    update_rate,                                                    # scanning rate (Hz)
    offset_pose,                                                    # offset pose on the body
    horizontal_samples,                                            # horizontal samples
    vertical_samples,                                              # vertical channels
    horizontal_fov,                                                # horizontal FOV (rad)
    max_vert_angle,                                                # max vertical angle (rad)
    min_vert_angle,                                                # min vertical angle (rad)
    100.0,                                                         # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                              # rectangular beam
    sample_radius,                                                 # sample radius
    divergence_angle,                                             # vertical divergence angle
    divergence_angle,                                             # horizontal divergence angle
    return_mode,                                                  # return mode
)
lidar.SetName("Lidar Sensor")                                       # name the sensor
lidar.SetLag(lag)                                                   # sensing lag
lidar.SetCollectionWindow(collection_time)                         # collection window

# Filter graph for post-processing the lidar data (ORDER MATTERS)
if noise_model == "CONST_NORMAL_XYZI":                              # Gaussian noise on the XYZI point cloud
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))  # mean/stdev noise model
elif noise_model == "NONE":                                        # no noise applied
    pass
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))  # raw depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar/"))               # save the XYZI point cloud per scan
lidar.PushFilter(sens.ChFilterXYZIAccess())                        # host access to XYZI buffer
manager.AddSensor(lidar)                                            # register the lidar with the manager

# Irrlicht window for real-time review of the static mesh scene
vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht visual system
vis.AttachSystem(mphysicalSystem)                                  # attach the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up camera convention
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Lidar Sensor")                                 # window title
vis.Initialize()                                                   # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # PyChrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # eye + look-at origin
vis.AddTypicalLights()                                             # standard lighting

# Simulation / orbit parameters
step_size = 1e-3                                                    # physics step size
end_time = 20.0                                                     # simulation end time
orbit_radius = 5                                                    # radius of the lidar orbit around the mesh
orbit_rate = 0.2                                                    # angular rate of the orbit (rad/s)
render_fps = 50.0                                                   # review render cadence
render_every = max(1, round(1.0 / (render_fps * step_size)))       # physics steps per rendered frame


while vis.Run() and mphysicalSystem.GetChTime() < end_time:        # main loop
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw the scene
    vis.EndScene()                                                 # end frame
    for _ in range(render_every):                                  # inner physics batch
        ch_time = mphysicalSystem.GetChTime()                      # current sim time
        lidar.SetOffsetPose(chrono.ChFramed(                       # orbit the lidar around the mesh
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                              -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()              # most recent XYZI point cloud
        if xyzi_buffer.HasData():                                  # only read after the first sensor tick
            xyzi_data = xyzi_buffer.GetXYZIData()                  # host-side XYZI array
            print('XYZI buffer recieved from lidar. Lidar resolution: {0}x{1}'
                  .format(xyzi_buffer.Width, xyzi_buffer.Height))   # print buffer resolution
            print('Max Value: {0}'.format(np.max(xyzi_data)))      # print max XYZI value
        manager.Update()                                           # update all sensors (once per step)
        mphysicalSystem.DoStepDynamics(step_size)                  # advance dynamics one step
        if mphysicalSystem.GetChTime() >= end_time:
            break
