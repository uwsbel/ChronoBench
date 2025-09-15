import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros
import os

def main():
    # Initialize Chrono system
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))

    # Create ground body with movement
    ground = chrono.ChBody()
    ground.SetFixed(False)
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    system.AddBody(ground)

    # Add visualization mesh to ground
    mesh = chrono.ChVisualShapeModelFile()
    mesh.SetFilename(chrono.GetChronoDataFile("models/cube.obj"))
    ground.AddVisualShape(mesh)

    # Create sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 100), chrono.ChColor(1, 1, 1), 5000)

    # Camera sensor
    camera_offset = chrono.ChVector3d(0.5, 0, 0.5)
    camera_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    camera = sens.ChCameraSensor(
        ground,
        30,  # update rate
        chrono.ChFrameD(camera_offset, camera_rot),
        1920,  # width
        1080,  # height
        chrono.CH_PI / 3  # FOV
    )
    camera.SetName("Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    # Lidar sensor
    lidar_offset = chrono.ChVector3d(0, 0.5, 0.5)
    lidar = sens.ChLidarSensor(
        ground,
        20,  # update rate
        chrono.ChFrameD(lidar_offset, chrono.Q_from_AngZ(chrono.CH_PI)),
        1800,  # horizontal samples
        16,    # vertical samples
        chrono.CH_PI,   # horizontal FOV
        chrono.CH_PI/6  # vertical FOV
    )
    lidar.SetName("Lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    manager.AddSensor(lidar)

    # IMU sensors (accelerometer, gyroscope, magnetometer)
    imu_offset = chrono.ChVector3d(0, 0, 0.5)
    imu_frame = chrono.ChFrameD(imu_offset)
    
    accel = sens.ChAccelerometerSensor(
        ground,
        1000,  # update rate
        imu_frame,
        sens.ChNoiseNormal(chrono.ChVector3d(0,0,0), chrono.ChVector3d(0.001,0.001,0.001))
    )
    accel.SetName("Accelerometer")
    manager.AddSensor(accel)

    gyro = sens.ChGyroscopeSensor(
        ground,
        1000,
        imu_frame,
        sens.ChNoiseNormal(chrono.ChVector3d(0,0,0), chrono.ChVector3d(0.001,0.001,0.001))
    )
    gyro.SetName("Gyroscope")
    manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(
        ground,
        100,
        imu_frame,
        sens.ChNoiseNormal(chrono.ChVector3d(0,0,0), chrono.ChVector3d(0.001,0.001,0.001)),
        chrono.ChVector3d(0, 0, -1)
    )
    mag.SetName("Magnetometer")
    manager.AddSensor(mag)

    # GPS sensor
    gps = sens.ChGPSSensor(
        ground,
        10,  # update rate
        chrono.ChFrameD(chrono.ChVector3d(0,0,0.2)),
        chrono.ChVector3d(0,0,0),
        sens.ChNoiseNormal(chrono.ChVector3d(0,0,0), chrono.ChVector3d(0.1,0.1,0.1))
    )
    gps.SetName("GPS")
    manager.AddSensor(gps)

    # ROS manager configuration
    ros_manager = chros.ChROSPublisher()
    ros_manager.RegisterSensor(camera, "camera_data")
    ros_manager.RegisterSensor(lidar, "lidar_data")
    ros_manager.RegisterSensor(accel, "imu/accelerometer")
    ros_manager.RegisterSensor(gyro, "imu/gyroscope")
    ros_manager.RegisterSensor(mag, "imu/magnetometer")
    ros_manager.RegisterSensor(gps, "gps_data")

    # Simulation loop
    time_step = 0.001
    realtime_step = time_step
    while system.GetChTime() < 10.0:  # Run for 10 seconds
        # Apply motion to ground body
        time = system.GetChTime()
        ground.SetPos(chrono.ChVector3d(0.1 * chrono.ChSineStep(time, 1, 0.5, 0), 0, 0))
        
        # Update sensors
        manager.Update()
        
        # Publish sensor data to ROS
        ros_manager.Process()
        
        # Advance simulation
        system.DoStepDynamics(time_step)
        
        # Real-time synchronization
        chrono.ChRealtimeStepTimer.Spin(time_step)

if __name__ == "__main__":
    main()