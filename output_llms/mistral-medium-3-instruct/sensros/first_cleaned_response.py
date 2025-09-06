import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import pychrono.ros as ros
import numpy as np
import time






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Sensor Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.SetSymbolScale(0.01)
vis.AssetBindAll()
vis.AssetUpdateAll()






ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
system.Add(ground)


ground_mesh = chrono.ChTriangleMeshConnected()
ground_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/flat_plane.obj"), True, True)
ground_mesh_shape = chrono.ChTriangleMeshShape()
ground_mesh_shape.SetMesh(ground_mesh)
ground_mesh_shape.SetName("ground_mesh")
ground.AddVisualShape(ground_mesh_shape)






platform = chrono.ChBody()
platform.SetMass(100)
platform.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
platform.SetPos(chrono.ChVectorD(0, 0.5, 0))
platform.SetCollide(True)
system.Add(platform)


platform_box = chrono.ChBoxShape()
platform_box.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 2)
platform_box.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
platform.AddVisualShape(platform_box)


motion = chrono.ChFunction_Oscillate(0.5, 2)  
platform.SetPos_dt(chrono.ChVectorD(0, 0, 0))
platform.SetRot_dt(chrono.ChQuaternionD(1, 0, 0, 0))






sensor_manager = sensors.ChSensorManager(system)


update_rate = 30


camera = sensors.ChCameraSensor(
    platform,                     
    update_rate,                  
    chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0), chrono.Q_from_AngX(chrono.CH_C_PI/4)),  
    640, 480,                     
    1.0                           
)
camera.SetName("camera_sensor")
camera.SetActive(True)
sensor_manager.AddSensor(camera)


lidar = sensors.ChLidarSensor(
    platform,
    update_rate,
    chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0.5), chrono.Q_from_AngX(0)),
    1.0,                          
    0.1,                          
    360                           
)
lidar.SetName("lidar_sensor")
lidar.SetActive(True)
sensor_manager.AddSensor(lidar)


gps = sensors.ChGpsSensor(
    platform,
    update_rate,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
gps.SetName("gps_sensor")
gps.SetActive(True)
sensor_manager.AddSensor(gps)


accel = sensors.ChAccelerometerSensor(
    platform,
    update_rate,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
accel.SetName("accelerometer")
accel.SetActive(True)
sensor_manager.AddSensor(accel)


gyro = sensors.ChGyroscopeSensor(
    platform,
    update_rate,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
gyro.SetName("gyroscope")
gyro.SetActive(True)
sensor_manager.AddSensor(gyro)


magnetometer = sensors.ChMagnetometerSensor(
    platform,
    update_rate,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
magnetometer.SetName("magnetometer")
magnetometer.SetActive(True)
sensor_manager.AddSensor(magnetometer)






ros_node = ros.ChRosNode("chrono_sensor_node")


camera_pub = ros_node.CreatePublisher("/chrono/camera", "sensor_msgs/Image")
lidar_pub = ros_node.CreatePublisher("/chrono/lidar", "sensor_msgs/LaserScan")
gps_pub = ros_node.CreatePublisher("/chrono/gps", "sensor_msgs/NavSatFix")
accel_pub = ros_node.CreatePublisher("/chrono/accel", "sensor_msgs/Imu")
gyro_pub = ros_node.CreatePublisher("/chrono/gyro", "sensor_msgs/Imu")
magnetometer_pub = ros_node.CreatePublisher("/chrono/mag", "sensor_msgs/MagneticField")






time_step = 0.01
sim_time = 0
max_sim_time = 20  


while sim_time < max_sim_time and vis.Run():
    
    motion_time = sim_time
    platform_pos = platform.GetPos()
    platform_pos.z = motion.Get_y(motion_time)
    platform.SetPos(platform_pos)

    
    sensor_manager.Update()

    
    if camera.IsActive():
        camera_data = camera.GetLastOutput()
        if camera_data:
            camera_pub.Publish(camera_data)

    if lidar.IsActive():
        lidar_data = lidar.GetLastOutput()
        if lidar_data:
            lidar_pub.Publish(lidar_data)

    if gps.IsActive():
        gps_data = gps.GetLastOutput()
        if gps_data:
            gps_pub.Publish(gps_data)

    if accel.IsActive():
        accel_data = accel.GetLastOutput()
        if accel_data:
            accel_pub.Publish(accel_data)

    if gyro.IsActive():
        gyro_data = gyro.GetLastOutput()
        if gyro_data:
            gyro_pub.Publish(gyro_data)

    if magnetometer.IsActive():
        mag_data = magnetometer.GetLastOutput()
        if mag_data:
            magnetometer_pub.Publish(mag_data)

    
    system.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sim_time += time_step

    
    ros_node.SpinOnce()

    
    time.sleep(time_step)


ros_node.Shutdown()