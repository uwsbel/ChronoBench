import pychrono as chrono
import pychrono.sensor as sens
import pychrono.vehicle  
import pychrono.irrlicht as irr

import time


try:
    import pychrono.sensor_ros as chrono_ros
except ImportError:
    chrono_ros = None
    print("[Warning] pychrono.sensor_ros not available. ROS integration disabled.")

def main():

    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath() or "./")  

    
    system = chrono.ChSystemNSC()

    
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(False)  
    ground.SetMass(1000)  
    ground.SetInertiaXX(chrono.ChVectorD(100, 100, 100))
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    ground.SetPos_dt(chrono.ChVectorD(0.5, 0.0, 0))  
    ground.SetBodyFixed(False)
    system.Add(ground)

    
    mesh_file = chrono.GetChronoDataFile('sensor/textures/box.obj')  
    
    try:
        ground_mesh = chrono.ChTriangleMeshShape()
        ground_mesh.SetMesh(chrono.ChTriangleMeshShape().LoadWavefrontMesh(mesh_file, True, False))
        ground_mesh.SetName("GroundMesh")
        ground_mesh.SetStatic(True)
        ground.GetAssets().push_back(ground_mesh)
    except Exception:
        
        box_shape = chrono.ChBoxShape()
        box_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
        ground.AddAsset(box_shape)

    
    application = irr.ChIrrApp(system, "PyChrono Multisensor Example", irr.dimension2du(1280, 720))
    application.AddTypicalSky()
    application.AddTypicalLogo()
    application.AddTypicalLights()
    application.AddTypicalCamera(chrono.ChVectorD(-10, -10, 10), chrono.ChVectorD(0, 0, 0))

    application.AssetBindAll()
    application.AssetUpdateAll()

    

    
    sensor_manager = sens.ChSensorManager(system)

    
    sensor_manager.SetChVisualSystem(application.GetVisualSystem())

    
    sensor_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.8))  

    
    time_step = 1.0 / 30       

    
    cam_width = 640
    cam_height = 480
    fov = 70

    camera = sens.ChCameraSensor(
        ground,
        time_step,
        sens.ChFrameD(chrono.ChVectorD(0, 0, 1.8)),  
        cam_width,
        cam_height,
        fov)
    camera.PushFilter(sens.ChFilterRGBA8Access())
    sensor_manager.AddSensor(camera)

    
    lidar = sens.ChLidarSensor(
        ground,
        time_step,
        sens.ChFrameD(chrono.ChVectorD(0, 0, 1.8)),
        100,     
        0.01,    
        360,     
        16,      
        -15,     
        15,      
        100.0)   
    sensor_manager.AddSensor(lidar)

    
    gps = sens.ChGPSSensor(
        ground,
        time_step,
        sens.ChFrameD(sensor_pose.GetPos()),
        1.0,    
        1.0)    
    sensor_manager.AddSensor(gps)

    
    accelerometer = sens.ChAccelerometerSensor(
        ground,
        time_step,
        sensor_pose,
        0.01)   
    sensor_manager.AddSensor(accelerometer)

    
    gyro = sens.ChGyroscopeSensor(
        ground,
        time_step,
        sensor_pose,
        0.001)  
    sensor_manager.AddSensor(gyro)

    
    magnetometer = sens.ChMagnetometerSensor(
        ground,
        time_step,
        sensor_pose,
        0.01)  
    sensor_manager.AddSensor(magnetometer)

    
    ros_manager = None
    if chrono_ros:
        ros_manager = chrono_ros.ChROS2Interface.Create()
        ros_manager.Initialize()

        
        ros_manager.RegisterSensor(camera, "/camera/image_raw")
        ros_manager.RegisterSensor(lidar, "/lidar/points")
        ros_manager.RegisterSensor(gps, "/gps/fix")
        ros_manager.RegisterSensor(accelerometer, "/imu/accel")
        ros_manager.RegisterSensor(gyro, "/imu/gyro")
        ros_manager.RegisterSensor(magnetometer, "/imu/mag")
    else:
        print("[Warning] ROS manager not initialized because pychrono.sensor_ros is unavailable.")

    
    sensor_manager.Initialize()

    
    simulation_step = 0.001  
    simulation_time = 10.0   

    t = 0
    last_time = time.time()

    
    application.SetTimestep(simulation_step)

    print("Starting simulation...")

    while t < simulation_time:
        
        start_sim_time = time.time()

        
        sensor_manager.Update()

        
        system.DoStepDynamics(simulation_step)

        
        if ros_manager:
            ros_manager.Update()

        
        if application.GetDevice().run():
            application.BeginScene()
            application.DrawAll()
            application.EndScene()
        else:
            
            break

        
        elapsed = time.time() - start_sim_time
        sleep_time = simulation_step - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        
        t += simulation_step

    print("Simulation ended.")


if __name__ == "__main__":
    main()