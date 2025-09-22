import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import os
import time


try:
    import pychrono.ros as ros
    ros_available = True
except ImportError:
    print("PyChrono ROS module not available. ROS functionality will be disabled.")
    ros_available = False

def main():
    
    print("Initialize the Chrono system")
    chrono_system = chrono.ChSystemNSC()
    chrono_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    chrono_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
    chrono_system.SetSolverMaxIterations(100)
    chrono_system.SetSolverForceTolerance(1e-8)
    
    
    print("Creating the ground body")
    ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
    ground.SetBodyFixed(False)  
    
    
    ground_motion = chrono.ChFunction_Sine(0, 0.5, 0.1)  
    ground.SetPos_dt(chrono.ChVectorD(0, 0, 0))  
    
    
    chrono_system.Add(ground)
    
    
    mesh_path = "path/to/your/mesh.obj"  
    if os.path.exists(mesh_path):
        mesh = chrono.ChTriangleMeshConnected()
        mesh.LoadWavefrontMesh(mesh_path)
        mesh_shape = chrono.ChTriangleMeshShape()
        mesh_shape.SetMesh(mesh)
        mesh_shape.SetName("terrain_mesh")
        mesh_shape.SetStatic(True)
        ground.AddAsset(mesh_shape)
    else:
        
        ground_texture = chrono.ChTexture()
        ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
        ground.AddAsset(ground_texture)
    
    
    ground_color = chrono.ChColorAsset()
    ground_color.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    ground.AddAsset(ground_color)
    
    
    sensor_update_rate = 100.0
    sensor_manager = sens.ChSensorManager(chrono_system)
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)
    
    
    sensor_offset = chrono.ChVectorD(0, 1.0, 0)  
    sensor_offset_pose = chrono.ChFrameD(sensor_offset)
    
    
    camera_update_rate = 30.0
    camera = sens.ChCameraSensor(
        ground,                             
        camera_update_rate,                 
        sensor_offset_pose,                 
        1280,                               
        720,                                
        3.14159 / 4                         
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(0.0)
    camera.SetCollectionWindow(0.0)
    camera.PushFilter(sens.ChFilterRGBA8Access())  
    camera.PushFilter(sens.ChFilterSave("camera/"))  
    sensor_manager.AddSensor(camera)
    
    
    lidar_update_rate = 10.0
    lidar = sens.ChLidarSensor(
        ground,                             
        lidar_update_rate,                  
        sensor_offset_pose,                 
        900,                                
        16,                                 
        3.14159,                            
        chrono.CH_C_PI / 12,                
        100                                 
    )
    lidar.SetName("LiDAR Sensor")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)
    lidar.PushFilter(sens.ChFilterPCfromDepth())  
    lidar.PushFilter(sens.ChFilterXYZIAccess())   
    lidar.PushFilter(sens.ChFilterVisualizePointCloud())  
    sensor_manager.AddSensor(lidar)
    
    
    gps_update_rate = 5.0
    gps = sens.ChGPSSensor(
        ground,                             
        gps_update_rate,                    
        sensor_offset_pose,                 
        sens.ChNoiseNone()                  
    )
    gps.SetName("GPS Sensor")
    gps.SetLag(0.0)
    gps.SetCollectionWindow(0.0)
    gps.PushFilter(sens.ChFilterGPSAccess())  
    sensor_manager.AddSensor(gps)
    
    
    imu_update_rate = 100.0
    
    
    accelerometer = sens.ChAccelerometerSensor(
        ground,                             
        imu_update_rate,                    
        sensor_offset_pose,                 
        sens.ChNoiseNone()                  
    )
    accelerometer.SetName("Accelerometer")
    accelerometer.SetLag(0.0)
    accelerometer.SetCollectionWindow(0.0)
    accelerometer.PushFilter(sens.ChFilterAccelAccess())  
    sensor_manager.AddSensor(accelerometer)
    
    
    gyroscope = sens.ChGyroscopeSensor(
        ground,                             
        imu_update_rate,                    
        sensor_offset_pose,                 
        sens.ChNoiseNone()                  
    )
    gyroscope.SetName("Gyroscope")
    gyroscope.SetLag(0.0)
    gyroscope.SetCollectionWindow(0.0)
    gyroscope.PushFilter(sens.ChFilterGyroAccess())  
    sensor_manager.AddSensor(gyroscope)
    
    
    magnetometer = sens.ChMagnetometerSensor(
        ground,                             
        imu_update_rate,                    
        sensor_offset_pose,                 
        sens.ChNoiseNone()                  
    )
    magnetometer.SetName("Magnetometer")
    magnetometer.SetLag(0.0)
    magnetometer.SetCollectionWindow(0.0)
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())  
    sensor_manager.AddSensor(magnetometer)
    
    
    if ros_available:
        print("Initializing ROS integration")
        ros_manager = ros.ChROSManager()
        
        
        ros_manager.AddSensor(camera, "/camera/image_raw")
        ros_manager.AddSensor(lidar, "/lidar/point_cloud")
        ros_manager.AddSensor(gps, "/gps/fix")
        ros_manager.AddSensor(accelerometer, "/imu/acceleration")
        ros_manager.AddSensor(gyroscope, "/imu/angular_velocity")
        ros_manager.AddSensor(magnetometer, "/imu/magnetic_field")
        
        
        ros_manager.Initialize()

    
    try:
        import pychrono.irrlicht as chronoirr
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(chrono_system)
        vis.SetWindowSize(1280, 720)
        vis.SetWindowTitle("PyChrono Sensor Demo")
        vis.Initialize()
        vis.AddCamera(chrono.ChVectorD(0, 5, -10))
        vis.AddTypicalLights()
        use_visualization = True
    except ImportError:
        print("Irrlicht visualization not available.")
        use_visualization = False

    
    step_size = 1e-3
    t_end = 20.0
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    
    print("Starting simulation...")
    t = 0
    
    while t < t_end:
        
        ground.SetPos_dt(chrono.ChVectorD(0, ground_motion.Get_y(t), 0))
        
        
        sensor_manager.Update()
        
        
        if ros_available:
            ros_manager.Update()
        
        
        if use_visualization and vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        elif use_visualization:
            print("Visualization window closed. Exiting.")
            break
        
        
        chrono_system.DoStepDynamics(step_size)
        
        
        realtime_timer.Spin(step_size)
        
        
        t += step_size
        
        
        if int(t * 100) % 100 == 0:
            print(f"Simulation time: {t:.2f} s")
    
    
    if use_visualization:
        vis.StopScene()
    
    if ros_available:
        ros_manager.Shutdown()
    
    print("Simulation completed.")

if __name__ == "__main__":
    main()