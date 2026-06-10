"""PyChrono sensor and ROS bridge scene.

This self-contained NSC simulation spins a named sensor carrier body carrying a
camera, 2D lidar, GPS, accelerometer, gyroscope, and magnetometer.  A visual
mesh body is explicitly added to the simulation so the lidar and camera have a
mesh target, and ROS2 handlers publish the body, TF, camera, lidar, GPS, and IMU
streams while the return value of the ROS manager update controls loop exit.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants ===
TIME_STEP = 0.005
SIM_END = 5.0
RENDER_FPS = 30.0
CAMERA_RATE = 30.0
LIDAR_RATE = 5.0
INERTIAL_RATE = 10.0
ROS_BODY_RATE = 25.0
LIDAR_HORIZONTAL_SAMPLES = 800
LIDAR_VERTICAL_SAMPLES = 1
MESH_TARGET_POS = chrono.ChVector3d(3.0, 0.0, 0.0)
GPS_REFERENCE = chrono.ChVector3d(-89.4, 43.07, 260.0)
SENSOR_OFFSET = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.25), chrono.QUNIT)
LIDAR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.35),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
CAMERA_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(-1.5, 0.0, 0.45),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


def build_scene():
    """Build the Chrono system, sensors, ROS handlers, and Irrlicht window."""
    # === System & gravity ===
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)
    sys.GetSolver().AsIterative().SetMaxIterations(50)

    # === Bodies ===
    ground_body = chrono.ChBody()
    ground_body.SetName("sensor_carrier")
    ground_body.SetMass(10.0)
    ground_body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
    ground_body.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    ground_body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 0.35))
    ground_body.EnableCollision(False)
    carrier_shape = chrono.ChVisualShapeSphere(0.18)
    carrier_shape.SetColor(chrono.ChColor(0.1, 0.35, 0.9))
    ground_body.AddVisualShape(carrier_shape)
    carrier_marker = chrono.ChVisualShapeBox(chrono.ChVector3d(0.8, 0.08, 0.08))
    carrier_marker.SetColor(chrono.ChColor(1.0, 0.85, 0.05))
    ground_body.AddVisualShape(carrier_marker, chrono.ChFramed(chrono.ChVector3d(0.45, 0.0, 0.0), chrono.QUNIT))
    sys.AddBody(ground_body)

    base_link = chrono.ChBody()
    base_link.SetName("base_link")
    base_link.SetFixed(True)
    base_link.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
    sys.AddBody(base_link)

    mesh_body = chrono.ChBody()
    mesh_body.SetName("mesh_body")
    mesh_body.SetFixed(True)
    mesh_body.SetPos(MESH_TARGET_POS)
    mesh_body.EnableCollision(False)
    mesh_shape = chrono.ChVisualShapeModelFile()
    mesh_shape.SetFilename(chrono.GetChronoDataFile("opensim/Rajagopal2015/r_foot.obj"))
    mesh_body.AddVisualShape(mesh_shape)
    sys.Add(mesh_body)

    # === Sensor manager & sensors ===
    manager = sens.ChSensorManager(sys)
    manager.scene.AddPointLight(
        chrono.ChVector3f(2.0, 2.5, 8.0),
        chrono.ChColor(1.0, 1.0, 1.0),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-3.0, -2.0, 5.0),
        chrono.ChColor(0.8, 0.8, 0.8),
        250.0,
    )

    camera = sens.ChCameraSensor(
        ground_body,
        CAMERA_RATE,
        CAMERA_OFFSET,
        1280,
        720,
        1.408,
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(0.0)
    camera.SetCollectionWindow(0.0)
    camera.PushFilter(sens.ChFilterVisualize(640, 360, "Camera Sensor"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
    manager.AddSensor(camera)

    lidar = sens.ChLidarSensor(
        ground_body,
        LIDAR_RATE,
        LIDAR_OFFSET,
        LIDAR_HORIZONTAL_SAMPLES,
        LIDAR_VERTICAL_SAMPLES,
        2.0 * chrono.CH_PI,
        0.0,
        0.0,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        2,
        0.003,
        0.003,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("2D Lidar Sensor")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
    lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_SAMPLES, "Raw 2D Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    gps = sens.ChGPSSensor(
        ground_body,
        INERTIAL_RATE,
        SENSOR_OFFSET,
        GPS_REFERENCE,
        sens.ChNoiseNone(),
    )
    gps.SetName("GPS Sensor")
    gps.SetLag(0.0)
    gps.SetCollectionWindow(0.0)
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    accelerometer = sens.ChAccelerometerSensor(ground_body, INERTIAL_RATE, SENSOR_OFFSET, sens.ChNoiseNone())
    accelerometer.SetName("Accelerometer Sensor")
    accelerometer.SetLag(0.0)
    accelerometer.SetCollectionWindow(0.0)
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)

    gyroscope = sens.ChGyroscopeSensor(ground_body, INERTIAL_RATE, SENSOR_OFFSET, sens.ChNoiseNone())
    gyroscope.SetName("Gyroscope Sensor")
    gyroscope.SetLag(0.0)
    gyroscope.SetCollectionWindow(0.0)
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)

    magnetometer = sens.ChMagnetometerSensor(ground_body, INERTIAL_RATE, SENSOR_OFFSET, sens.ChNoiseNone(), GPS_REFERENCE)
    magnetometer.SetName("Magnetometer Sensor")
    magnetometer.SetLag(0.0)
    magnetometer.SetCollectionWindow(0.0)
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magnetometer)

    # === ROS bridge ===
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(ROS_BODY_RATE, ground_body, "~/output/body"))
    tf_handler = chros.ChROSTFHandler(ROS_BODY_RATE)
    tf_handler.AddTransform(base_link, base_link.GetName(), ground_body, ground_body.GetName())
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAMERA_RATE, camera, "~/output/camera/data"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(LIDAR_RATE, lidar, "~/output/lidar/data"))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(accelerometer, "~/output/accelerometer/data")
    gyro_handler = chros.ChROSGyroscopeHandler(gyroscope, "~/output/gyroscope/data")
    mag_handler = chros.ChROSMagnetometerHandler(magnetometer, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(acc_handler)
    ros_manager.RegisterHandler(gyro_handler)
    ros_manager.RegisterHandler(mag_handler)
    imu_handler = chros.ChROSIMUHandler(INERTIAL_RATE, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)
    ros_manager.Initialize()

    # === Visualization ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Sensor ROS Bridge")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5.0, -4.0, 3.0), chrono.ChVector3d(0.75, 0.0, 0.2))
    vis.AddTypicalLights()
    vis.AddGrid(
        0.5,
        0.5,
        16,
        16,
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -0.2), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )

    return sys, ground_body, manager, ros_manager, vis


def main():
    """Run the sensor and ROS simulation."""
    sys, ground_body, manager, ros_manager, vis = build_scene()
    carrier = ground_body  # cache: fetched once and reused while logging
    frame = 0


    # === Main loop ===
    try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = sys.GetChTime()
                manager.Update()
                if not ros_manager.Update(time, TIME_STEP):
                    return
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"Simulation failed: {exc}")
        raise

    # === Post-processing ===


if __name__ == "__main__":
    main()
