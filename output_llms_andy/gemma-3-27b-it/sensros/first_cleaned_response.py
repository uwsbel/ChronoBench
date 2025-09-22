import pychrono as chrono
import pychrono.sensor as sensor
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirrlicht
import math
import time


chrono.SetChronoDataPath("../data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasy()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetShape(chrono.ChBoxShape())
ground.SetMass(1000)
system.Add(ground)


ground_asset = chrono.ChVisualShapeEasy(ground, True)
ground_asset.SetFile("terrain.obj") 
ground_asset.SetColored(chrono.ChColor(0.5, 0.5, 0.5))


sensor_manager = sensor.ChSensorManager(system)



camera = sensor.ChCameraSensor()
camera.Setup(system, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 60, 0.1, 100)
camera.SetFOV(math.pi / 3)
sensor_manager.AddSensor(camera)


lidar = sensor.ChLidarSensor()
lidar.Setup(system, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 360, 0.1, 10)
lidar.SetResolution(1024)
sensor_manager.AddSensor(lidar)


gps = sensor.ChGPSSensor()
gps.Setup(system, ground)
sensor_manager.AddSensor(gps)


accelerometer = sensor.ChAccelerometerSensor()
accelerometer.Setup(system, ground)
sensor_manager.AddSensor(accelerometer)


gyroscope = sensor.ChGyroscopeSensor()
gyroscope.Setup(system, ground)
sensor_manager.AddSensor(gyroscope)


magnetometer = sensor.ChMagnetometerSensor()
magnetometer.Setup(system, ground)
sensor_manager.AddSensor(magnetometer)


try:
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, PointCloud2, NavSatFix, FluidPressure, Imu, MagneticField
    import sensor_msgs.point_cloud2 as pc2
    import numpy as np

    
    rospy.init_node('chrono_sensors', anonymous=True)

    
    camera_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    lidar_pub = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    accelerometer_pub = rospy.Publisher('/imu/accelerometer', Imu, queue_size=10)
    gyroscope_pub = rospy.Publisher('/imu/gyroscope', Imu, queue_size=10)
    magnetometer_pub = rospy.Publisher('/imu/magnetic_field', MagneticField, queue_size=10)

    def publish_sensor_data():
        
        if camera.HasNewData():
            img = camera.GetImage()
            img_msg = Image()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.height = img.GetHeight()
            img_msg.width = img.GetWidth()
            img_msg.encoding = "bgr8"
            img_msg.data = img.GetData()
            camera_pub.publish(img_msg)

        
        if lidar.HasNewData():
            points = lidar.GetPointCloud()
            cloud_msg = pc2.create_cloud(rospy.Header(), points)
            lidar_pub.publish(cloud_msg)

        
        if gps.HasNewData():
            gps_data = gps.GetLocation()
            gps_msg = NavSatFix()
            gps_msg.header.stamp = rospy.Time.now()
            gps_msg.latitude = gps_data.latitude
            gps_msg.longitude = gps_data.longitude
            gps_msg.altitude = gps_data.altitude
            gps_pub.publish(gps_msg)

        
        if accelerometer.HasNewData():
            acc_data = accelerometer.GetSensorData()
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.linear_acceleration.x = acc_data.x
            imu_msg.linear_acceleration.y = acc_data.y
            imu_msg.linear_acceleration.z = acc_data.z
            accelerometer_pub.publish(imu_msg)

        
        if gyroscope.HasNewData():
            gyro_data = gyroscope.GetSensorData()
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.angular_velocity.x = gyro_data.x
            imu_msg.angular_velocity.y = gyro_data.y
            imu_msg.angular_velocity.z = gyro_data.z
            gyroscope_pub.publish(imu_msg)

        
        if magnetometer.HasNewData():
            mag_data = magnetometer.GetSensorData()
            mag_msg = MagneticField()
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.magnetic_field.x = mag_data.x
            mag_msg.magnetic_field.y = mag_data.y
            mag_msg.magnetic_field.z = mag_data.z
            magnetometer_pub.publish(mag_msg)

except ImportError:
    print("ROS not found. Skipping ROS integration.")
    rospy = None


time_step = 0.005
time_now = 0
while True:
    
    sensor_manager.Update(time_step)

    
    if rospy is not None:
        publish_sensor_data()

    
    system.DoStepDynamics(time_step)
    time_now += time_step

    
    print(f"Time: {time_now:.2f}")

    
    if time_now > 10:
        break