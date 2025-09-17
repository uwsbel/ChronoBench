importchrono
import rospy
from sensor_msgs.msg import Image, LaserScan, NavSatFix, Imu, MagneticField


rospy.init_node('chrono_sensor_node')


system = chrono.ChSystem()
system.SetCollisionDetection(chrono.ChCollisionSystem())


ground = chrono.ChBody()
ground.SetMass(0)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetDimensions(chrono.ChVectorD(10, 10, 1))
ground.GetCollisionModel().AddBox(chrono.ChVectorD(5, 5, 0.5))
ground.GetVisualizationAsset().SetObject(chrono.ChBox(5, 5, 0.5))
system.AddBody(ground)


sensor_manager = chrono.ChSensorManager(system)


camera = chrono.ChSensorCamera()
camera.SetPosition(chrono.ChVectorD(0, 0, 2))
camera.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
sensor_manager.AddSensor(camera)

lidar = chrono.ChSensorLidar()
lidar.SetPosition(chrono.ChVectorD(0, 0, 2))
lidar.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
lidar.SetHorizontalAngleRange(chrono.ChRange<double>(-1.5708, 1.5708))
lidar.SetVerticalAngleRange(chrono.ChRange<double>(-0.7854, 0.7854))
lidar.SetRange(chrono.ChRange<double>(0.1, 10))
sensor_manager.AddSensor(lidar)

gps = chrono.ChSensorGPS()
gps.SetPosition(chrono.ChVectorD(0, 0, 2))
sensor_manager.AddSensor(gps)

accelerometer = chrono.ChSensorAccelerometer()
accelerometer.SetPosition(chrono.ChVectorD(0, 0, 2))
sensor_manager.AddSensor(accelerometer)

gyroscope = chrono.ChSensorGyroscope()
gyroscope.SetPosition(chrono.ChVectorD(0, 0, 2))
sensor_manager.AddSensor(gyroscope)

magnetometer = chrono.ChSensorMagnetometer()
magnetometer.SetPosition(chrono.ChVectorD(0, 0, 2))
sensor_manager.AddSensor(magnetometer)


camera_pub = rospy.Publisher('camera/image', Image, queue_size=1)
lidar_pub = rospy.Publisher('lidar/scan', LaserScan, queue_size=1)
gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=1)
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)
mag_pub = rospy.Publisher('mag/data', MagneticField, queue_size=1)


def publish_camera(data):
    msg = Image()
    msg.width = data.width
    msg.height = data.height
    msg.encoding = 'rgb8'
    msg.data = data.GetPixels()
    camera_pub.publish(msg)

def publish_lidar(data):
    msg = LaserScan()
    msg.angle_min = -1.5708
    msg.angle_max = 1.5708
    msg.angle_increment = 0.01745
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = data.GetRanges()
    lidar_pub.publish(msg)

def publish_gps(data):
    msg = NavSatFix()
    msg.latitude = data.latitude
    msg.longitude = data.longitude
    msg.altitude = data.altitude
    gps_pub.publish(msg)

def publish_imu(data):
    msg = Imu()
    msg.linear_acceleration = data.acceleration
    msg.angular_velocity = data.angular_velocity
    msg.orientation = data.orientation
    imu_pub.publish(msg)

def publish_mag(data):
    msg = MagneticField()
    msg.magnetic_field = data.magnetic_field
    mag_pub.publish(msg)


sensor_manager.RegisterCallback(camera, publish_camera)
sensor_manager.RegisterCallback(lidar, publish_lidar)
sensor_manager.RegisterCallback(gps, publish_gps)
sensor_manager.RegisterCallback(accelerometer, publish_imu)
sensor_manager.RegisterCallback(gyroscope, publish_imu)
sensor_manager.RegisterCallback(magnetometer, publish_mag)


prev_time = rospy.Time.now().to_sec()
while not rospy.is_shutdown():
    
    sensor_manager.Update()

    
    system.DoStepDynamics(chrono.ChRealtimeStep())

    
    curr_time = rospy.Time.now().to_sec()
    elapsed = curr_time - prev_time
    if elapsed < 0.016:  
        rospy.sleep(0.016 - elapsed)
    prev_time = curr_time

    
    if chrono.GetKeyboard().KeyIsDown(chrono.KEY_Q):
        break