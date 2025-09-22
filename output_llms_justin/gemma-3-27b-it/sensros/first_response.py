import pychrono as chrono
import pychrono.sensor as sensor
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np
import time

# Initialize the PyChrono system
chrono.SetChronoDataPath("./")  # Replace with your ChronoDataPath if needed
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create a ground body
body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(body_ground)

# Add a box shape to the ground for visualization
box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().SetMinMax(chrono.ChVectorD(-10, -1, -10), chrono.ChVectorD(10, 0, 10))
body_ground.AddAsset(box_shape)

# Add a collision shape to the ground
body_ground.AddAsset(chrono.ChCollisionShape())
body_ground.SetCollide(True)

# Create a moving platform (optional, for sensor testing)
body_platform = chrono.ChBodyEasy()
body_platform.SetPos(chrono.ChVectorD(0, 1, 0))
body_platform.SetMass(1)
system.Add(body_platform)

# Add a box shape to the platform for visualization
platform_shape = chrono.ChBoxShape()
platform_shape.GetBoxGeometry().SetMinMax(chrono.ChVectorD(-0.5, -0.1, -0.5), chrono.ChVectorD(0.5, 0.1, 0.5))
body_platform.AddAsset(platform_shape)
body_platform.SetCollide(True)
body_platform.AddAsset(chrono.ChCollisionShape())

# Create a sensor manager
sensor_manager = sensor.ChSensorManager()

# Create sensors
# Camera sensor
camera = sensor.ChCameraSensor()
camera.Setup(body_platform, chrono.ChVectorD(0, 0.1, 0), chrono.ChVectorD(0, 0, 0), 60)
sensor_manager.AddSensor(camera)

# Lidar sensor
lidar = sensor.ChLidarSensor()
lidar.Setup(body_platform, chrono.ChVectorD(0, 0.1, 0), chrono.ChVectorD(0, 0, 0), 360, 0.1)
sensor_manager.AddSensor(lidar)

# GPS sensor
gps = sensor.ChGPSSensor()
gps.Setup(body_platform)
sensor_manager.AddSensor(gps)

# Accelerometer sensor
accelerometer = sensor.ChAccelerometerSensor()
accelerometer.Setup(body_platform)
sensor_manager.AddSensor(accelerometer)

# Gyroscope sensor
gyroscope = sensor.ChGyroscopeSensor()
gyroscope.Setup(body_platform)
sensor_manager.AddSensor(gyroscope)

# Magnetometer sensor
magnetometer = sensor.ChMagnetometerSensor()
magnetometer.Setup(body_platform)
sensor_manager.AddSensor(magnetometer)


# ROS integration (requires ROS environment)
try:
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
    import sensor_msgs.point_cloud2 as pc2
    import ros_numpy

    # Initialize ROS node
    rospy.init_node('pychrono_sensors', anonymous=True)

    # Publishers
    camera_pub = rospy.Publisher('/camera_image', Image, queue_size=10)
    lidar_pub = rospy.Publisher('/lidar_points', PointCloud2, queue_size=10)
    gps_pub = rospy.Publisher('/gps_fix', NavSatFix, queue_size=10)
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)

    def publish_camera_data(image_data):
        img_msg = Image()
        img_msg.height = image_data.shape[0]
        img_msg.width = image_data.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.data = image_data.flatten().tolist()
        camera_pub.publish(img_msg)

    def publish_lidar_data(points):
        points_np = np.array(points)
        fields = [
            ( "x", 0, ros_numpy.np_float32, 1),
            ( "y", 4, ros_numpy.np_float32, 1),
            ( "z", 8, ros_numpy.np_float32, 1),
            ( "intensity", 12, ros_numpy.np_float32, 1)
        ]

        cloud = pc2.create_cloud(points_np, header=None, fields=fields)
        lidar_pub.publish(cloud)

    def publish_gps_data(latitude, longitude, altitude):
        gps_msg = NavSatFix()
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude
        gps_pub.publish(gps_msg)

    def publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.magnetic_field.x = mag_x
        imu_msg.magnetic_field.y = mag_y
        imu_msg.magnetic_field.z = mag_z
        imu_pub.publish(imu_msg)

except ImportError:
    print("ROS not found. Skipping ROS integration.")
    camera_pub = None
    lidar_pub = None
    gps_pub = None
    imu_pub = None

# Simulation loop
timestep = 0.01
time_elapsed = 0

while time_elapsed < 10:
    system.DoStepDynamics(timestep)
    sensor_manager.Update()

    # Get sensor data and publish to ROS
    if camera_pub:
        image_data = camera.GetImage()
        publish_camera_data(image_data)

    if lidar_pub:
        points = lidar.GetPoints()
        publish_lidar_data(points)

    if gps_pub:
        latitude, longitude, altitude = gps.GetCoordinates()
        publish_gps_data(latitude, longitude, altitude)

    if imu_pub:
        accel_x, accel_y, accel_z = accelerometer.GetAcceleration()
        gyro_x, gyro_y, gyro_z = gyroscope.GetAngularVelocity()
        mag_x, mag_y, mag_z = magnetometer.GetMagneticField()
        publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z)

    time_elapsed += timestep
    #time.sleep(timestep)  # Optional: Add a small delay for real-time execution

print("Simulation finished.")