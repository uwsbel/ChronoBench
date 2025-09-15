importchrono
import rospy
from sensor_msgs.msg import Image, LaserScan, NavSatFix, Imu, MagneticField
import numpy as np


rospy.init_node('chrono_sensor_simulation', anonymous=True)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetCollisionEnable(True)


ground = chrono.ChBody()
ground.SetMass(0)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
ground.GetCollisionModel().SetFamily(1)
ground.GetCollisionModel().AddBox(chrono.ChCollisionBox(chrono.ChVectorD(10, 10, 10)))
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 10, 10))
ground.GetAssets().push_back(ground_shape)


sensor_manager = chrono.ChSensorManager()


camera = chrono.ChSensorCamera()
camera.SetName("camera_sensor")
camera.SetPosition(chrono.ChVectorD(0, 1, 0))
camera.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
sensor_manager.AddSensor(camera, ground)

lidar = chrono.ChSensorLidar()
lidar.SetName("lidar_sensor")
lidar.SetPosition(chrono.ChVectorD(0, 1, 0))
lidar.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
lidar.SetRange(50)
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetVerticalAngleRange(chrono.ChRange<double>(-45, 45))
sensor_manager.AddSensor(lidar, ground)

gps = chrono.ChSensorGPS()
gps.SetName("gps_sensor")
gps.SetPosition(chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(gps, ground)

accelerometer = chrono.ChSensorAccelerometer()
accelerometer.SetName("accelerometer_sensor")
accelerometer.SetPosition(chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(accelerometer, ground)

gyroscope = chrono.ChSensorGyroscope()
gyroscope.SetName("gyroscope_sensor")
gyroscope.SetPosition(chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(gyroscope, ground)

magnetometer = chrono.ChSensorMagnetometer()
magnetometer.SetName("magnetometer_sensor")
magnetometer.SetPosition(chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(magnetometer, ground)


camera_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
lidar_pub = rospy.Publisher('/lidar/scan', LaserScan, queue_size=1)
gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
accel_pub = rospy.Publisher('/accel', Imu, queue_size=1)
gyro_pub = rospy.Publisher('/gyro', Imu, queue_size=1)
mag_pub = rospy.Publisher('/mag', MagneticField, queue_size=1)


def publish_camera(data):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.width = data.width
    msg.height = data.height
    msg.encoding = "rgb8"
    msg.data = data.GetPixels()
    camera_pub.publish(msg)

def publish_lidar(data):
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.angle_min = -np.pi/2
    msg.angle_max = np.pi/2
    msg.angle_increment = 0.1
    msg.range_min = 0.0
    msg.range_max = 50.0
    msg.ranges = data.GetRanges()
    lidar_pub.publish(msg)

def publish_gps(data):
    msg = NavSatFix()
    msg.header.stamp = rospy.Time.now()
    msg.latitude = data.GetLatitude()
    msg.longitude = data.GetLongitude()
    msg.altitude = data.GetAltitude()
    gps_pub.publish(msg)

def publish_accel(data):
    msg = Imu()
    msg.header.stamp = rospy.Time.now()
    msg.linear_acceleration.x = data.GetAcceleration().x
    msg.linear_acceleration.y = data.GetAcceleration().y
    msg.linear_acceleration.z = data.GetAcceleration().z
    accel_pub.publish(msg)

def publish_gyro(data):
    msg = Imu()
    msg.header.stamp = rospy.Time.now()
    msg.angular_velocity.x = data.GetAngularVelocity().x
    msg.angular_velocity.y = data.GetAngularVelocity().y
    msg.angular_velocity.z = data.GetAngularVelocity().z
    gyro_pub.publish(msg)

def publish_mag(data):
    msg = MagneticField()
    msg.header.stamp = rospy.Time.now()
    msg.magnetic_field.x = data.GetMagneticField().x
    msg.magnetic_field.y = data.GetMagneticField().y
    msg.magnetic_field.z = data.GetMagneticField().z
    mag_pub.publish(msg)

sensor_manager.RegisterSensorCallback(camera.GetName(), publish_camera)
sensor_manager.RegisterSensorCallback(lidar.GetName(), publish_lidar)
sensor_manager.RegisterSensorCallback(gps.GetName(), publish_gps)
sensor_manager.RegisterSensorCallback(accelerometer.GetName(), publish_accel)
sensor_manager.RegisterSensorCallback(gyroscope.GetName(), publish_gyro)
sensor_manager.RegisterSensorCallback(magnetometer.GetName(), publish_mag)


simulation_time = 0
real_time = 0
step_size = 0.01

while not rospy.is_shutdown():
    
    sensor_manager.Update()

    
    system.DoStepDynamics(step_size)
    simulation_time += step_size

    
    real_time = rospy.Time.now().to_sec()
    if real_time - simulation_time > step_size:
        simulation_time = real_time

    
    if keyboard.is_pressed('q'):
        break


sensor_manager.RemoveAllSensors()
system.Clear()