import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import time
import os

def main():
    # ===== 1. Initialize Chrono System =====
    chrono.SetChronoDataPath("")  # Set default data path
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))

    # ===== 2. Create Ground Body with Motion =====
    ground = chrono.ChBody()
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground.SetFixed(True)
    system.Add(ground)

    # Add visualization mesh to ground
    mesh_path = chrono.GetChronoDataFile("models/cube.obj")
    mesh = chrono.ChVisualShapeModelFile(mesh_path)
    mesh.SetScale(chrono.ChVector3d(0.2, 0.2, 0.2))
    ground.AddVisualShape(mesh, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.1)))

    # ===== 3. Create Sensor Manager =====
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 1000)

    # ===== 4. Create and Attach Sensors to Ground =====
    # Camera Sensor
    cam_offset = chrono.ChFramed(chrono.ChVector3d(0.5, 0, 0.5), chrono.Q_ROTATE_Y_TO_Z)
    camera = sens.ChCameraSensor(
        ground,           # Attached body
        30,               # Update rate [Hz]
        cam_offset,       # Offset pose
        1280,             # Image width
        720,              # Image height
        chrono.CH_PI / 3  # Horizontal FOV
    )
    camera.SetName("Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    # Lidar Sensor
    lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 1))
    lidar = sens.ChLidarSensor(
        ground,            # Attached body
        10,                # Update rate [Hz]
        lidar_offset,      # Offset pose
        360,               # Horizontal samples
        180,               # Vertical samples
        chrono.CH_PI,      # Horizontal FOV
        chrono.CH_PI / 4,  # Vertical FOV
        0.1,               # Min range
        100.0              # Max range
    )
    lidar.SetName("Lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    manager.AddSensor(lidar)

    # IMU Sensors (Accelerometer, Gyroscope, Magnetometer)
    imu_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.3))
    accel = sens.ChAccelerometerSensor(ground, 100, imu_offset)
    gyro = sens.ChGyroscopeSensor(ground, 100, imu_offset)
    mag = sens.ChMagnetometerSensor(ground, 100, imu_offset)
    
    manager.AddSensor(accel)
    manager.AddSensor(gyro)
    manager.AddSensor(mag)

    # GPS Sensor
    gps_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5))
    gps = sens.ChGPSSensor(ground, 10, gps_offset)
    manager.AddSensor(gps)

    # ===== 5. Initialize ROS Manager =====
    ros_manager = sens.ChROSPublisher()
    ros_manager.RegisterSensor(camera, "/camera")
    ros_manager.RegisterSensor(lidar, "/lidar")
    ros_manager.RegisterSensor(accel, "/accel")
    ros_manager.RegisterSensor(gyro, "/gyro")
    ros_manager.RegisterSensor(mag, "/mag")
    ros_manager.RegisterSensor(gps, "/gps")

    # ===== 6. Create Visualization System =====
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Sensors Demo")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2, 2, 1), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # ===== 7. Simulation Loop =====
    step_size = 0.001
    realtime_step = True

    while vis.Run():
        # Update sensors
        manager.Update()
        
        # Update ROS publishers
        ros_manager.Update()
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Maintain real-time synchronization
        if realtime_step:
            time.sleep(step_size)

if __name__ == "__main__":
    main()