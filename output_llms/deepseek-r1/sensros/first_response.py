import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import math
import time

def main():
    # =============================================================================
    # 1. Initialize PyChrono environment
    # =============================================================================
    chrono.SetChronoDataPath("C:/path/to/chrono/data/")  # Set your Chrono data path
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))

    # =============================================================================
    # 2. Create ground body and motion
    # =============================================================================
    ground = chrono.ChBody()
    ground.SetFixed(False)
    ground.SetMass(1)
    ground.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    system.Add(ground)

    # Add visualization mesh
    mesh = chrono.ChVisualShapeBox(0.2, 0.2, 0.1)
    mesh.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
    ground.AddVisualShape(mesh)

    # Motion function (vertical oscillation)
    motion_func = chrono.ChFunctionSine(0.5, 0.5)  # amplitude=0.5m, frequency=0.5Hz
    motion = chrono.ChLinkMotorLinPosition()
    motion.SetMotionFunction(motion_func)
    motion.Initialize(ground, system.GetGroundBody(), 
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    system.Add(motion)

    # =============================================================================
    # 3. Create sensor manager and sensors
    # =============================================================================
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2.5, 5), chrono.ChColor(1, 1, 1), 1000)

    # Camera sensor
    cam_offset = chrono.ChVector3d(0.1, 0, 0.1)
    cam_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    camera = sens.ChCameraSensor(
        ground,
        30,  # update rate [Hz]
        chrono.ChFrameD(cam_offset, cam_rot),
        1920,  # width
        1080,  # height
        chrono.CH_PI / 3  # FOV
    )
    camera.SetName("Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    # Lidar sensor
    lidar_offset = chrono.ChVector3d(0, 0.1, 0.1)
    lidar = sens.ChLidarSensor(
        ground,
        10,  # update rate
        chrono.ChFrameD(lidar_offset, chrono.Q_from_AngZ(chrono.CH_PI)),
        1800,  # horizontal samples
        30,    # vertical samples
        chrono.CH_PI,  # horizontal FOV
        chrono.CH_PI/6  # vertical FOV
    )
    lidar.SetName("Lidar")
    lidar.PushFilter(sens.ChFilterAccess())
    manager.AddSensor(lidar)

    # IMU sensors (accelerometer, gyroscope, magnetometer)
    imu_offset = chrono.ChVector3d(0, 0, 0.05)
    imu_frame = chrono.ChFrameD(imu_offset)
    
    accel = sens.ChAccelerometerSensor(
        ground,
        100,  # update rate
        imu_frame,
        chrono.ChVector3d(0, 0, -9.81)  # offset gravity
    )
    accel.SetName("Accelerometer")
    manager.AddSensor(accel)

    gyro = sens.ChGyroscopeSensor(
        ground,
        100,  # update rate
        imu_frame
    )
    gyro.SetName("Gyroscope")
    manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(
        ground,
        100,  # update rate
        imu_frame,
        chrono.ChVector3d(0, 1, 0)  # local magnetic field
    )
    mag.SetName("Magnetometer")
    manager.AddSensor(mag)

    # GPS sensor
    gps = sens.ChGPSSensor(
        ground,
        10,  # update rate
        chrono.ChFrameD(chrono.ChVector3d(0,0,0.1)),
        chrono.ChVector3d(0, 0, 0),  # reference location
        0.1  # noise stdev
    )
    gps.SetName("GPS")
    manager.AddSensor(gps)

    # =============================================================================
    # 4. Set up ROS integration
    # =============================================================================
    ros_manager = sens.ChROSPublisher()
    ros_manager.RegisterSensor(camera, "/sensors/camera")
    ros_manager.RegisterSensor(lidar, "/sensors/lidar")
    ros_manager.RegisterSensor(accel, "/sensors/accel")
    ros_manager.RegisterSensor(gyro, "/sensors/gyro")
    ros_manager.RegisterSensor(mag, "/sensors/mag")
    ros_manager.RegisterSensor(gps, "/sensors/gps")

    # =============================================================================
    # 5. Create visualization and simulation loop
    # =============================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Sensors Demo")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1, 1, 1), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Simulation parameters
    step_size = 0.001
    realtime_step = True
    frame = 0

    while vis.Run():
        # Update sensor positions to follow moving body
        for sensor in [camera, lidar, accel, gyro, mag, gps]:
            sensor.PushUpdate()

        # Update ROS data
        ros_manager.Update(step_size, frame)

        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Maintain real-time pacing
        if realtime_step:
            time.sleep(step_size)
        
        frame += 1

if __name__ == "__main__":
    main()