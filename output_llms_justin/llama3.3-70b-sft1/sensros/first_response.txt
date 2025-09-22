import pychrono as ch
import pychrono.sensor as sens

import pychrono.ros as chros

import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0,0,-9.81))

    # ----------------------------------------
    # add a mesh object to make it interesting
    # ----------------------------------------
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile(
        "vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(2))

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)
    sys.Add(mesh_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(sys)
    intensity = 1.0
    manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        mesh_body,              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                    # camera's horizontal field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # ------------------------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        mesh_body,              # body lidar is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        horizontal_samples,     # number of horizontal samples
        vertical_samples,       # number of vertical channels
        horizontal_fov,         # horizontal field of view
        max_vert_angle,
        min_vert_angle,         # vertical field of view
        100  # max lidar range
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetReturnMode(sens.LidarReturnMode_STRONGEST_RETURN)

    # --------------------------------------------------------
    # Create a GPS and add it to the sensor manager
    # --------------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    gps = sens.ChGPSSensor(mesh_body,              # body gps is attached to
                           update_rate,            # update rate in Hz
                           offset_pose,            # offset pose
                           gps_reference,
                           nois_deviation)
    gps.SetName("GPS Sensor")
    gps.SetLag(lag)

    # --------------------------------------------------------
    # Create an accelerometer and add it to the sensor manager
    # --------------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    acc = sens.ChAccelerometerSensor(mesh_body,              # body acc is attached to
                                     update_rate,            # update rate in Hz
                                     offset_pose,            # offset pose
                                     noise_none)
    acc.SetName("Accelerometer Sensor")
    acc.SetLag(lag)

    # --------------------------------------------------------
    # Create a gyroscope and add it to the sensor manager
    # --------------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    gyro = sens.ChGyroscopeSensor(mesh_body,              # body gyro is attached to
                                  update_rate,            # update rate in Hz
                                  offset_pose,            # offset pose
                                  noise_none)
    gyro.SetName("Gyroscope Sensor")
    gyro.SetLag(lag)

    # --------------------------------------------------------
    # Create a magnetometer and add it to the sensor manager
    # --------------------------------------------------------
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    mag = sens.ChMagnetometerSensor(mesh_body,              # body mag is attached to
                                    update_rate,            # update rate in Hz
                                    offset_pose,            # offset pose
                                    gps_reference,
                                    noise_none)
    mag.SetName("Magnetometer Sensor")
    mag.SetLag(lag)

    # -----------------------
    # Add sensors to manager
    # -----------------------
    manager.AddSensor(cam)
    manager.AddSensor(lidar)
    manager.AddSensor(gps)
    manager.AddSensor(acc)
    manager.AddSensor(gyro)
    manager.AddSensor(mag)

    # ----------------------------------------------
    # Create a filter graph for post-processing
    # ----------------------------------------------
    # Access the filter graph for the first sensor
    filter_graph = manager.GetFilterGraph(cam.GetName())

    # Create a filter to change the image to grayscale
    gray_filter = sens.ChFilterGrayscale()
    filter_graph.AddFilter(gray_filter)

    # Create a filter to resize the image
    resize_filter = sens.ChFilterImageResize()
    resize_filter.SetNewWidth(256)
    resize_filter.SetNewHeight(256)
    filter_graph.AddFilter(resize_filter)

    # ---------------------
    # Create a ROS manager
    # ---------------------
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # ----------------------------------------------
    # Create a ROS handler for the camera sensor
    # ----------------------------------------------
    # Since we want to publish the filtered image
    # we must access the filter graph for the sensor
    # and register the handler for the last filter
    # in the graph
    cam_handler = chros.ChROSCameraHandler(
        filter_graph.GetOutput(),  # output to publish
        "~/output/camera/data/image"  # ros topic
    )
    cam_handler.SetFps(5)
    cam_handler.SetImageEncoding("rgb8")
    ros_manager.RegisterHandler(cam_handler)

    # ------------------------------------------------
    # Create a ROS handler for the lidar sensor
    # ------------------------------------------------
    lidar_handler = chros.ChROSLidarHandler(
        lidar,  # sensor to publish
        "~/output/lidar/data/pointcloud"  # ros topic
    )
    lidar_handler.SetFps(5)
    ros_manager.RegisterHandler(lidar_handler)

    # ----------------------------------------------
    # Create a ROS handler for the gps sensor
    # ----------------------------------------------
    gps_handler = chros.ChROSGPSHandler(
        gps,  # sensor to publish
        "~/output/gps/data"  # ros topic
    )
    gps_handler.SetFps(5)
    ros_manager.RegisterHandler(gps_handler)

    # ------------------------------------------------
    # Create a ROS handler for the accelerometer
    # ------------------------------------------------
    acc_handler = chros.ChROSAccelerometerHandler(
        acc,  # sensor to publish
        "~/output/accelerometer/data"  # ros topic
    )
    acc_handler.SetFps(5)
    ros_manager.RegisterHandler(acc_handler)

    # ------------------------------------------------
    # Create a ROS handler for the gyroscope sensor
    # ------------------------------------------------
    gyro_handler = chros.ChROSGyroscopeHandler(
        gyro,  # sensor to publish
        "~/output/gyroscope/data"  # ros topic
    )
    gyro_handler.SetFps(5)
    ros_manager.RegisterHandler(gyro_handler)

    # ------------------------------------------------
    # Create a ROS handler for the magnetometer sensor
    # ------------------------------------------------
    mag_handler = chros.ChROSMagnetometerHandler(
        mag,  # sensor to publish
        "~/output/magnetometer/data"  # ros topic
    )
    mag_handler.SetFps(5)
    ros_manager.RegisterHandler(mag_handler)

    # ---------------
    # Simulate system
    # ---------------
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    sys.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    t1 = time.time()

    orbit_radius = 20
    orbit_rate = 0.2
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    sys.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    render_time = 0

    while sys.GetChTime() < end_time:
        # Updates
        t = sys.GetChTime()

        # Set the ground body's position and orientation
        driver_inputs = ch.ChVector3d(
            orbit_radius * math.cos(t * orbit_rate),
            orbit_radius * math.sin(t * orbit_rate),
            0.0
        )
        mesh_body.SetPos(ch.ChVector3d(
            orbit_radius * math.cos(t * orbit_rate),
            orbit_radius * math.sin(t * orbit_rate),
            0.1
        ))
        mesh_body.SetRot(ch.QuatFromAngleAxis(
            t * orbit_rate, ch.ChVector3d(0, 0, 1)))

        # Access the sensor's filter graph and update
        # the output for each filter in the graph
        manager.Update()

        # Advance simulation for one timestep for all sensors
        manager.Advance(step_size)

        # Advance state of entire system (containing all bodies/sensors)
        sys.DoStepDynamics(step_size)

        # Enforce real-time simulation (block until next update)
        render_time = time.time()
        sys_time = sys.GetChTime()
        time_diff = (sys_time - render_time) * step_size
        if time_diff > 0:
            time.sleep(time_diff)

    print("Sim time:", end_time, "Wall time:", time.time()-t1)

# -----------------
# Sensor parameters
# -----------------


# Update rate in Hz
update_rate = 5.0

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408

# Lag time
lag = 0

# Exposure (in seconds) for camera
exposure_time = 0

# Number of horizontal and vertical samples
horizontal_samples = 4500
vertical_samples = 32

# Horizontal and vertical field of view for lidar
horizontal_fov = 2 * ch.CH_PI  # 360 degrees
max_vert_angle = ch.CH_PI / 12
min_vert_angle = -ch.CH_PI / 6

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# GPS reference point
gps_reference = ch.ChVector3d(0, 0, 0)

# GPS noise model
# (none, gaussian, constant)
gps_noise_model = "NONE"

# Noise deviation for GPS noise model
nois_deviation = 0

# Noise model attached to accelerometer and gyroscope
# (none, gaussian, constant)
noise_none = "NONE"

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 100.0

# Output directory
out_dir = "SENSOR_OUTPUT/"

main()