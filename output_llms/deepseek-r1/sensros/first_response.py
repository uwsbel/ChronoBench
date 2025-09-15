import os
import math
import time
import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], ''))

def main():
    # Initialize system
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

    # Create moving ground body
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    ground_body.SetName("MovingGround")
    system.Add(ground_body)

    # Add vertical motion using prismatic joint
    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(ground_body, system.GetBody(0), True,
                     chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                     chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    motor.SetMotionFunction(chrono.ChFunction_Sine(0.5, 0.5, 0))
    system.Add(motor)

    # Initialize sensor manager
    sensor_manager = sens.ChSensorManager(system)
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 0), chrono.ChColor(1, 1, 1), 500)

    #---------------------------
    # Sensor Configuration
    #---------------------------
    
    # Camera Sensor
    camera_frame = chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.5), 
                                 chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    camera = sens.ChCameraSensor(
        ground_body, 30, camera_frame,
        1280, 720, math.radians(90)
    )
    camera.SetName("Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    sensor_manager.AddSensor(camera)

    # Lidar Sensor
    lidar_frame = chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 0.5), 
                                chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        ground_body, 10, lidar_frame,
        1000, 100, math.radians(360), math.radians(30),
        100.0, sens.LidarReturnMode_DUAL_RETURN
    )
    lidar.SetName("Lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    sensor_manager.AddSensor(lidar)

    # GPS Sensor
    gps_frame = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.2))
    gps = sens.ChGPSSensor(
        ground_body, 10, gps_frame,
        chrono.ChVectorD(-117.27, 32.82, 0)  # UCSD coordinates
    )
    sensor_manager.AddSensor(gps)

    # IMU Sensors
    imu_frame = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))
    noise_model = sens.ChNoiseNormal(chrono.ChVectorD(0), chrono.ChVectorD(0.001))
    
    accelerometer = sens.ChAccelerometerSensor(
        ground_body, 100, imu_frame, noise_model
    )
    sensor_manager.AddSensor(accelerometer)

    gyroscope = sens.ChGyroscopeSensor(
        ground_body, 100, imu_frame, noise_model
    )
    sensor_manager.AddSensor(gyroscope)

    magnetometer = sens.ChMagnetometerSensor(
        ground_body, 10, imu_frame, noise_model, chrono.ChVectorD(0, 0, 1)
    )
    sensor_manager.AddSensor(magnetometer)

    #---------------------------
    # ROS Integration
    #---------------------------
    ros_manager = sens.ChROSPublisher()
    ros_manager.RegisterSensor(camera, "/camera/image")
    ros_manager.RegisterSensor(lidar, "/lidar/pointcloud")
    ros_manager.RegisterSensor(gps, "/gps/data")
    ros_manager.RegisterSensor(accelerometer, "/imu/accelerometer")
    ros_manager.RegisterSensor(gyroscope, "/imu/gyroscope")
    ros_manager.RegisterSensor(magnetometer, "/imu/magnetometer")
    ros_manager.Initialize()

    #---------------------------
    # Visualization Setup
    #---------------------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Sensor Demo")
    vis.Initialize()
    vis.AddCamera(chrono.ChVectorD(3, 3, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    #---------------------------
    # Simulation Loop
    #---------------------------
    step_size = 0.01
    end_time = 20.0

    while system.GetChTime() < end_time:
        if vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Update sensors and simulation
        sensor_manager.Update()
        system.DoStepDynamics(step_size)
        
        # Publish ROS data
        ros_manager.Update(system.GetChTime())

        # Real-time synchronization
        time.sleep(step_size - (time.time() % step_size))

if __name__ == "__main__":
    main()