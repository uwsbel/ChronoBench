import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr 


import os
import math
import time




try:
    import rospy
    from sensor_msgs.msg import Image as RosImage
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import NavSatFix
    from sensor_msgs.msg import Imu as RosImu 
    from geometry_msgs.msg import Vector3Stamped 
    ros_available = True
except ImportError:
    print("ROS (rospy) not found. ROS publishing will be disabled.")
    ros_available = False

def main():
    print("Chrono version:", chrono.CHRONO_VERSION)
    print("Chrono::Sensor version:", sens.GetVersion())

    
    
    
    my_system = chrono.ChSystemNSC()
    my_system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    
    
    ground_mat = chrono.ChContactMaterialNSC() 

    ground_body = chrono.ChBodyEasyBox(40, 4, 40, 1000, True, True, ground_mat)
    ground_body.SetPos(chrono.ChVector3d(0, -2, 0)) 
    ground_body.SetFixed(False) 
    my_system.Add(ground_body)

    
    
    
    
    

    
    ground_velocity = chrono.ChVector3d(0.5, 0, 0.2) 
    ground_body.SetPosDt(ground_velocity)
    

    
    
    
    sensor_manager = sens.ChSensorManager(my_system)
    sensor_manager.SetRayTracer(sens.ChRaytracer.DefaultType) 
    sensor_manager.SetVerbose(False) 
    
    


    
    
    
    if ros_available:
        try:
            
            rospy.get_master().getSystemState()
            print("ROS Master detected. Initializing ROS node.")
            
            
            if not rospy.core.is_initialized():
                 rospy.init_node('chrono_sensor_simulation', anonymous=True)
            ros_manager = sens.ChROSManager()
        except Exception as e:
            print(f"Could not connect to ROS Master or initialize node: {e}")
            print("ROS publishing will be disabled for this run.")
            ros_available = False 
    else:
        print("ROS functionalities are disabled as rospy is not available.")

    
    
    
    
    update_rate = 30.0  

    
    
    
    cam_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 1.0, 0), 
        chrono.QuatFromAngleAxis(chrono.CH_PI / 12.0, chrono.ChVector3d(0, 1, 0)) 
    )
    camera = sens.ChCameraSensor(
        ground_body,                                
        update_rate,                                
        cam_offset_pose,                            
        1280,                                       
        720,                                        
        chrono.CH_PI / 3.0,                         
        
    )
    camera.SetName("CameraSensor")
    camera.SetLag(0.02) 
    camera.SetCollectionWindow(0) 
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed")) 
    camera.PushFilter(sens.ChFilterRGBA8Access()) 
    sensor_manager.AddSensor(camera)
    if ros_available:
        ros_manager.RegisterSensor(camera, "/chrono_ros/camera/image_raw", message_type=RosImage)


    
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.5, 1.5, 0), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)) 
    )
    lidar = sens.ChLidarSensor(
        ground_body,
        10.0, 
        lidar_offset_pose,
        900,                                         
        32,                                          
        2.0 * chrono.CH_PI,                          
        (chrono.CH_PI / 6.0), 
        
        
        100.0,                                       
        sens.LidarBeamShape.RECTANGULAR,             
        2,                                           
        1,                                           
        sens.LidarNoiseModel.NOISE_NONE             
    )
    lidar.SetName("LidarSensor")
    lidar.SetLag(0.01)
    lidar.SetCollectionWindow(0.001)
    lidar.PushFilter(sens.ChFilterPCfromDepth()) 
    lidar.PushFilter(sens.ChFilterXYZIAccess()) 
    sensor_manager.AddSensor(lidar)
    if ros_available:
        ros_manager.RegisterSensor(lidar, "/chrono_ros/lidar/point_cloud", message_type=PointCloud2)

    
    
    
    
    gps_reference_point = chrono.ChVector3d(42.0565, -87.6753, 200.0) 
    gps_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0, 1.0, -0.5), 
        chrono.QUNIT
    )
    gps = sens.ChGPSSensor(
        ground_body,
        5.0, 
        gps_offset_pose,
        gps_reference_point,
        sens.ChNoiseNone() 
    )
    gps.SetName("GPSSensor")
    gps.PushFilter(sens.ChFilterGPSAccess())
    sensor_manager.AddSensor(gps)
    if ros_available:
        ros_manager.RegisterSensor(gps, "/chrono_ros/gps/fix", message_type=NavSatFix)


    
    imu_update_rate = 100.0 
    imu_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.1, 0.8, 0.1), 
        chrono.QUNIT
    )

    
    accel = sens.ChAccelerometerSensor(
        ground_body,
        imu_update_rate,
        imu_offset_pose,
        sens.ChNoiseNone() 
    )
    accel.SetName("Accelerometer")
    accel.PushFilter(sens.ChFilterAccelAccess())
    sensor_manager.AddSensor(accel)
    if ros_available:
        
        ros_manager.RegisterSensor(accel, "/chrono_ros/imu/accelerometer", message_type=Vector3Stamped)

    
    gyro = sens.ChGyroscopeSensor(
        ground_body,
        imu_update_rate,
        imu_offset_pose,
        sens.ChNoiseNone()
    )
    gyro.SetName("Gyroscope")
    gyro.PushFilter(sens.ChFilterGyroAccess())
    sensor_manager.AddSensor(gyro)
    if ros_available:
        ros_manager.RegisterSensor(gyro, "/chrono_ros/imu/gyroscope", message_type=Vector3Stamped)

    
    
    
    
    mag_reference_field_enu = chrono.ChVector3d(20e-6, 5e-6, -35e-6) 
    mag = sens.ChMagnetometerSensor(
        ground_body,
        imu_update_rate,
        imu_offset_pose,
        sens.ChNoiseNone(),
        gps_reference_point, 
        mag_reference_field_enu 
    )
    mag.SetName("Magnetometer")
    mag.PushFilter(sens.ChFilterMagnetAccess())
    sensor_manager.AddSensor(mag)
    if ros_available:
        ros_manager.RegisterSensor(mag, "/chrono_ros/imu/magnetometer", message_type=Vector3Stamped)

    
    
    
    
    
    

    
    
    
    use_irrlicht_viz = True
    if use_irrlicht_viz:
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(my_system)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("Chrono Sensor Demo with ROS")
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(10, 10, -15), chrono.ChVector3d(0, 0, 0)) 
        vis.AddTypicalLights()
        
        

    
    
    
    time_step = 1e-3  
    sim_time = 0.0
    end_time = 60.0   

    
    realtime_timer = chrono.ChRealtimeStepTimer()
    realtime_timer.SetStealTime(True) 

    print("\nStarting simulation loop...")
    if ros_available:
        print("ROS Topics should be available (e.g., /chrono_ros/camera/image_raw).")
        print("Use 'rostopic list' and 'rostopic echo <topic_name>' to inspect.")
        print("Or use RViz to visualize point clouds and camera images.")
    else:
        print("ROS publishing is disabled.")

    try:
        if use_irrlicht_viz:
            while vis.Run() and sim_time < end_time:
                vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1))
                vis.Render()
                
                
                
                vis.EndScene()

                
                sensor_manager.Update()

                
                if ros_available:
                    ros_manager.Update()

                
                my_system.DoStepDynamics(time_step)
                sim_time += time_step

                
                realtime_timer.Spin(time_step)

                if sim_time % 1 < time_step : 
                     print(f"Sim time: {sim_time:.2f} s")

        else: 
            while sim_time < end_time:
                
                sensor_manager.Update()

                
                if ros_available:
                    ros_manager.Update()

                
                my_system.DoStepDynamics(time_step)
                sim_time += time_step

                
                realtime_timer.Spin(time_step)

                if sim_time % 1 < time_step : 
                     print(f"Sim time: {sim_time:.2f} s, Ground pos: {ground_body.GetPos().x:.2f}, {ground_body.GetPos().y:.2f}, {ground_body.GetPos().z:.2f}")

                
                if ros_available and rospy.is_shutdown():
                    print("ROS shutdown requested. Exiting simulation.")
                    break
    
    except Exception as e:
        print(f"An error occurred during the simulation loop: {e}")
    finally:
        print("Simulation finished.")
        if use_irrlicht_viz and vis:
            vis.GetDevice().closeDevice()
        
        
        
        del sensor_manager
        print("Sensor manager deleted.")
        if ros_available:
            
            pass


if __name__ == "__main__":
    
    
    
    
    sens.SetSensorDataPath(os.path.join(chrono.GetChronoDataPath(), "sensor"))

    main()