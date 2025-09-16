import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header

# Initialize PyChrono environment and core components
physics = chrono.ChSystemNSC()
physics.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create ground body
ground = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
physics.Add(ground)

# Set ground body motion
ground.SetPos_dt(chrono.ChVectorD(0, 0.1, 0))

# Create sensor manager
sensor_manager = chrono.ChSensorManager()
physics.Add(sensor_manager)

# Add camera sensor
camera = chrono.ChCameraSensor(chrono.ChVectorD(0, 0, 0.5), chrono.ChVectorD(0, 0, -1))
camera.SetName("camera")
sensor_manager.AddSensor(camera)

# Add lidar sensor
lidar = chrono.ChLidarSensor(chrono.ChVectorD(0, 0, 0.5), 100, 1000, 45, 0.02)
lidar.SetName("lidar")
sensor_manager.AddSensor(lidar)

# Add GPS sensor
gps = chrono.ChGPSSensor(chrono.ChVectorD(0, 0, 0.5))
gps.SetName("gps")
sensor_manager.AddSensor(gps)

# Add accelerometer sensor
accelerometer = chrono.ChAccelerometerSensor(chrono.ChVectorD(0, 0, 0.5))
accelerometer.SetName("accelerometer")
sensor_manager.AddSensor(accelerometer)

# Add gyroscope sensor
gyroscope = chrono.ChGyroscopeSensor(chrono.ChVectorD(0, 0, 0.5))
gyroscope.SetName("gyroscope")
sensor_manager.AddSensor(gyroscope)

# Add magnetometer sensor
magnetometer = chrono.ChMagnetometerSensor(chrono.ChVectorD(0, 0, 0.5))
magnetometer.SetName("magnetometer")
sensor_manager.AddSensor(magnetometer)

# Initialize ROS node and publishers
rospy.init_node('pychrono_sensors')
camera_pub = rospy.Publisher('/camera', Image, queue_size=10)
lidar_pub = rospy.Publisher('/lidar', PointCloud2, queue_size=10)
gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10)
accel_pub = rospy.Publisher('/accelerometer', Vector3Stamped, queue_size=10)
gyro_pub = rospy.Publisher('/gyroscope', Vector3Stamped, queue_size=10)
mag_pub = rospy.Publisher('/magnetometer', Vector3Stamped, queue_size=10)

# Simulation loop
application = chronoirr.ChIrrApp(physics, 'PyChrono Sensors', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chrono.ChVectorD(0, -2, 0.5))
application.AddLightWithShadow(chrono.ChVectorD(2, 2, 2), 0.4, 5, 5, 50, 128)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()

    # Update sensors
    sensor_manager.Update()

    # Publish data to ROS topics
    camera_data = sensor_manager.GetSensor("camera").Get_Data()
    camera_msg = Image()
    camera_msg.height = camera_data.GetHeight()
    camera_msg.width = camera_data.GetWidth()
    camera_msg.encoding = "rgb8"
    camera_msg.is_bigendian = False
    camera_msg.step = camera_data.GetWidth() * 3
    camera_msg.data = np.array(camera_data.GetData(), dtype=np.uint8).flatten().tolist()
    camera_pub.publish(camera_msg)

    lidar_data = sensor_manager.GetSensor("lidar").Get_Data()
    lidar_msg = PointCloud2()
    lidar_msg.header.frame_id = "lidar"
    lidar_msg.height = 1
    lidar_msg.width = len(lidar_data)
    lidar_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]
    lidar_msg.is_bigendian = False
    lidar_msg.point_step = 16
    lidar_msg.row_step = lidar_msg.point_step * len(lidar_data)
    lidar_msg.data = np.array(lidar_data, dtype=np.float32).flatten().tolist()
    lidar_pub.publish(lidar_msg)

    gps_data = sensor_manager.GetSensor("gps").Get_Data()
    gps_msg = NavSatFix()
    gps_msg.header.frame_id = "gps"
    gps_msg.latitude = gps_data.GetLatitude()
    gps_msg.longitude = gps_data.GetLongitude()
    gps_msg.altitude = gps_data.GetAltitude()
    gps_pub.publish(gps_msg)

    accelerometer_data = sensor_manager.GetSensor("accelerometer").Get_Data()
    accelerometer_msg = Vector3Stamped()
    accelerometer_msg.header.frame_id = "accelerometer"
    accelerometer_msg.vector.x = accelerometer_data.GetAcceleration().x
    accelerometer_msg.vector.y = accelerometer_data.GetAcceleration().y
    accelerometer_msg.vector.z = accelerometer_data.GetAcceleration().z
    accel_pub.publish(accelerometer_msg)

    gyroscope_data = sensor_manager.GetSensor("gyroscope").Get_Data()
    gyroscope_msg = Vector3Stamped()
    gyroscope_msg.header.frame_id = "gyroscope"
    gyroscope_msg.vector.x = gyroscope_data.GetAngularVelocity().x
    gyroscope_msg.vector.y = gyroscope_data.GetAngularVelocity().y
    gyroscope_msg.vector.z = gyroscope_data.GetAngularVelocity().z
    gyro_pub.publish(gyroscope_msg)

    magnetometer_data = sensor_manager.GetSensor("magnetometer").Get_Data()
    magnetometer_msg = Vector3Stamped()
    magnetometer_msg.header.frame_id = "magnetometer"
    magnetometer_msg.vector.x = magnetometer_data.GetMagneticField().x
    magnetometer_msg.vector.y = magnetometer_data.GetMagneticField().y
    magnetometer_msg.vector.z = magnetometer_data.GetMagneticField().z
    mag_pub.publish(magnetometer_msg)

    application.EndScene()

application.Wait()