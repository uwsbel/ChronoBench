"""PyChrono NSC sensor/ROS demo with a moving instrumented body.

The simulation creates one Chrono system with a dynamic mesh-decorated ground
body carrying camera, lidar, GPS, accelerometer, gyroscope, and magnetometer
sensors. A ROS2 Python manager publishes clock, body, TF, and sensor messages
while Irrlicht shows the moving body and the sensor manager updates off-screen
sensor streams.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants === simulation timing, body geometry, and sensor rates
time_step = 0.01
sim_end = 3.0
render_fps = 15.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
body_size = chrono.ChVector3d(2.0, 0.35, 1.0)
body_mass = 80.0
body_inertia = chrono.ChVector3d(8.0, 8.0, 8.0)
body_initial_pos = chrono.ChVector3d(0.0, 0.8, 0.0)
body_ang_vel = chrono.ChVector3d(0.0, 0.25, 0.55)
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)


def make_sensor_offset(x, y, z, pitch=0.0):
    """Return a local sensor frame mounted on the moving body."""
    return chrono.ChFramed(
        chrono.ChVector3d(x, y, z),
        chrono.QuatFromAngleAxis(pitch, chrono.ChVector3d(0, 1, 0)),
    )


def main():
    # === System & body === one NSC system with a collision body visible to sensors
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.6)
    material.SetRestitution(0.0)

    ground_body = chrono.ChBody()
    ground_body.SetName("ground_body")
    ground_body.SetMass(body_mass)
    ground_body.SetInertiaXX(body_inertia)
    ground_body.SetPos(body_initial_pos)
    ground_body.SetAngVelParent(body_ang_vel)
    ground_body.EnableCollision(True)
    ground_body.AddCollisionShape(chrono.ChCollisionShapeBox(material, body_size))

    body_visual = chrono.ChVisualShapeBox(body_size)
    body_visual.SetColor(chrono.ChColor(0.15, 0.35, 0.8))
    ground_body.AddVisualShape(body_visual)

    mesh_visual = chrono.ChVisualShapeModelFile()
    mesh_visual.SetFilename(chrono.GetChronoDataFile("models/cube.obj"))
    mesh_visual.SetColor(chrono.ChColor(0.95, 0.55, 0.1))
    ground_body.AddVisualShape(
        mesh_visual,
        chrono.ChFramed(chrono.ChVector3d(0.0, 0.45, 0.0), chrono.QUNIT),
    )
    sys.AddBody(ground_body)

    base_link = chrono.ChBody()
    base_link.SetName("base_link")
    base_link.SetFixed(True)
    sys.AddBody(base_link)

    tracked_body = ground_body  # cache: moving sensor carrier reused every step

    # === Sensor manager === camera, lidar, GPS, accelerometer, gyro, and magnetometer
    manager = sens.ChSensorManager(sys)
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(-8, -4, 20), chrono.ChColor(1, 1, 1), 200.0
    )

    camera = sens.ChCameraSensor(
        tracked_body,
        30.0,
        make_sensor_offset(-3.0, 0.0, 1.0, 0.15),
        640,
        360,
        1.408,
    )
    camera.SetName("camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0)
    camera.PushFilter(sens.ChFilterRGBA8Access())
    camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
    manager.AddSensor(camera)

    lidar = sens.ChLidarSensor(
        tracked_body,
        5.0,
        make_sensor_offset(-1.0, 0.0, 0.75, 0.0),
        200,
        4,
        2 * chrono.CH_PI,
        chrono.CH_PI / 12,
        -chrono.CH_PI / 12,
        50.0,
        sens.LidarBeamShape_RECTANGULAR,
        2,
        0.003,
        0.003,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("lidar")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / 5.0)
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    imu_pose = make_sensor_offset(0.0, 0.25, 0.0, 0.0)
    noise = sens.ChNoiseNone()

    gps = sens.ChGPSSensor(tracked_body, 10.0, imu_pose, gps_reference, noise)
    gps.SetName("gps")
    gps.SetLag(0)
    gps.SetCollectionWindow(0)
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    accelerometer = sens.ChAccelerometerSensor(tracked_body, 100.0, imu_pose, noise)
    accelerometer.SetName("accelerometer")
    accelerometer.SetLag(0)
    accelerometer.SetCollectionWindow(0)
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)

    gyroscope = sens.ChGyroscopeSensor(tracked_body, 100.0, imu_pose, noise)
    gyroscope.SetName("gyroscope")
    gyroscope.SetLag(0)
    gyroscope.SetCollectionWindow(0)
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)

    magnetometer = sens.ChMagnetometerSensor(tracked_body, 100.0, imu_pose, noise, gps_reference)
    magnetometer.SetName("magnetometer")
    magnetometer.SetLag(0)
    magnetometer.SetCollectionWindow(0)
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magnetometer)

    # === ROS bridge === handlers publish the moving body, TF, and sensor streams
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(30.0, tracked_body, "~/output/body/data"))

    tf_handler = chros.ChROSTFHandler(30.0)
    tf_handler.AddTransform(base_link, base_link.GetName(), tracked_body, tracked_body.GetName())
    ros_manager.RegisterHandler(tf_handler)

    ros_manager.RegisterHandler(chros.ChROSCameraHandler(30.0, camera, "~/output/camera/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/pointcloud"))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

    acc_handler = chros.ChROSAccelerometerHandler(accelerometer, "~/output/accelerometer/data")
    gyro_handler = chros.ChROSGyroscopeHandler(gyroscope, "~/output/gyroscope/data")
    mag_handler = chros.ChROSMagnetometerHandler(magnetometer, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(acc_handler)
    ros_manager.RegisterHandler(gyro_handler)
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100.0, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)
    ros_manager.Initialize()

    # === Visualization === Irrlicht window built unconditionally for review and live display
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Sensor ROS Demo")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5, 3, 5), chrono.ChVector3d(0, 0.8, 0))
    vis.AddTypicalLights()
    vis.AddGrid(
        0.5,
        0.5,
        30,
        30,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        chrono.ChColor(0.4, 0.4, 0.4),
    )

    realtime_timer = chrono.ChRealtimeStepTimer()
    frame = 0

    # === Main loop === update sensors and ROS every physics step, render per frame
    try:
            while vis.Run() and sys.GetChTime() < sim_end:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                for _ in range(render_every):
                    time = sys.GetChTime()  # cache: reused for ROS, logging, and timing
                    pos = tracked_body.GetPos()  # cache: body pose reused in review log
                    ang_vel = tracked_body.GetAngVelParent()  # cache: body rate reused in review log
                    manager.Update()
                    if not ros_manager.Update(time, time_step):
                        return
                    sys.DoStepDynamics(time_step)
                    realtime_timer.Spin(time_step)
                    if sys.GetChTime() >= sim_end:
                        break
    except (OSError, IOError) as exc:  # disk or permission failure during review logging
        print(f"I/O failure while writing review data: {exc}")
        raise
    except (RuntimeError, ValueError) as exc:  # Chrono/ROS runtime failure or bad numeric state
        print(f"Simulation failure: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
