import pychrono as chrono
import pychrono.sensor as sensors
import pychrono.irrlicht as chronoirr
import pyros as ros  # Assuming the ROS module is named pyros

# Initialize Chrono system
my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetMaxItersSolverSpeed(100)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create moving ground body
ground = chrono.ChBody()
ground.SetBodyFixed(False)  # Enable motion
ground.SetMass(1000)  # Set mass for dynamic behavior
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetPos_dt(chrono.ChVectorD(1, 0, 0))  # Initial velocity along x-axis
my_system.Add(ground)

# Add a mesh for visualization (simple box)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 10, 0.1))
ground.AddAsset(ground_shape)

# Initialize sensor manager
sensor_mgr = sensors.ChSensorManager()
sensor_mgr.SetCoordinateSystem(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))

# Initialize ROS node and manager
ros_node = ros.Ros()
ros_node.init('chronosim')
ros_mgr = sensors.ChRosSensorManager()
ros_mgr.SetRosInterface(ros_node)

# Configure sensors
# Camera
cam = sensors.ChCameraSensor()
cam.SetName("camera")
cam.SetPos(chrono.ChVectorD(0, 0, 1))  # Position relative to ground
cam.SetDir(chrono.ChVectorD(0, -1, 0))  # Look down
cam.SetFov(60)
cam.SetImageWidth(640)
cam.SetImageHeight(480)
cam.SetPositionRelative(ground)
sensor_mgr.AddSensor(cam)
ros_mgr.RegisterSensor(cam, '/camera/image', 'sensor_msgs/Image')

# Lidar
lidar = sensors.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPos(chrono.ChVectorD(0, 0, 1))
lidar.SetDir(chrono.ChVectorD(0, -1, 0))
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetRange(50)
lidar.SetPositionRelative(ground)
sensor_mgr.AddSensor(lidar)
ros_mgr.RegisterSensor(lidar, '/lidar/points', 'sensor_msgs/PointCloud2')

# GPS
gps = sensors.ChGpsSensor()
gps.SetName("gps")
gps.SetUpdateRate(1.0/10)  # 10Hz update rate
gps.SetPositionRelative(ground)
sensor_mgr.AddSensor(gps)
ros_mgr.RegisterSensor(gps, '/gps/fix', 'sensor_msgs/NavSatFix')

# IMU (accelerometer, gyroscope)
imu = sensors.ChImuSensor()
imu.SetName("imu")
imu.SetPositionRelative(ground)
sensor_mgr.AddSensor(imu)
ros_mgr.RegisterSensor(imu, '/imu/data', 'sensor_msgs/Imu')

# Magnetometer (custom implementation using body orientation)
class MagnetometerSensor(sensors.ChSensor):
    def __init__(self, body):
        super().__init__()
        self.body = body
        self.magnetic_field = chrono.ChVectorD(0, 0, 100000)  # Earth's magnetic field (arbitrary units)

    def Update(self):
        rotation = self.body.GetFrame_REF_to_abs().Get_A_Rotation()
        magnetic_body = rotation.Transpose() * self.magnetic_field
        self.SetData(magnetic_body)

mag = MagnetometerSensor(ground)
mag.SetName("magnetometer")
sensor_mgr.AddSensor(mag)
ros_mgr.RegisterSensor(mag, '/magnetometer/data', 'sensor_msgs/MagneticField')

# Simulation loop parameters
step_size = 0.01
sim_time = 10.0

# Start simulation loop
start_time = time.time()
while my_system.GetChTime() < sim_time:
    # Update sensors
    sensor_mgr.Update()
    
    # Advance simulation
    my_system.DoStepDynamics(step_size)
    
    # Maintain ground movement (constant velocity)
    ground.SetPos_dt(chrono.ChVectorD(1, 0, 0))  # Set velocity each step
    
    # Handle ROS messages
    ros_node.spin_once()
    
    # Real-time control
    elapsed = time.time() - start_time
    sleep_time = step_size - (time.time() - start_time)
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        print("Simulation is falling behind real-time")

print("Simulation completed.")