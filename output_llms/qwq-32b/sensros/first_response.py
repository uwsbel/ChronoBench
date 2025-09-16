import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as cr

# Initialize Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create ground body (dynamic)
ground_body = chrono.ChBody()
ground_body.SetIdentifier(1)
ground_body.SetBodyFixed(False)  # Allow movement
ground_body.SetMass(100)  # Mass in kg
ground_body.SetPos(chrono.ChVectorD(0, 0, 0))
ground_body.SetInertiaXX(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1))
sys.Add(ground_body)

# Add visualization mesh to ground
ground_shape = chrono.ChBox(2, 0.5, 2)
ground_body.AddAsset(chrono.ChTriangleMeshShape())
ground_body.GetAsset(0).SetMesh(ground_shape)
ground_body.GetAsset(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))

# Initialize sensor manager
sensor_mgr = sens.ChSensorManager(sys)

# Configure sensors
# Camera
camera = sens.ChCameraSensor()
camera.SetName("camera")
camera.SetPosition(chrono.ChVectorD(0, 0.5, 0))  # Relative to ground
camera.SetRotation(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
camera.SetImageWidth(640)
camera.SetImageHeight(480)
camera.SetFOV(60)
camera.SetPositionRelative(True)
camera.SetBody(ground_body)
sensor_mgr.AddSensor(camera)

# Lidar
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0.5, 0))
lidar.SetRotation(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
lidar.SetRange(50)
lidar.SetHorizontalResolution(0.5)
lidar.SetVerticalResolution(0.5)
lidar.SetPointsPerSecond(100000)
lidar.SetBody(ground_body)
sensor_mgr.AddSensor(lidar)

# GPS
gps = sens.ChGPSSensor()
gps.SetName("gps")
gps.SetBody(ground_body)
sensor_mgr.AddSensor(gps)

# Accelerometer
accelerometer = sens.ChAccelerometerSensor()
accelerometer.SetName("accelerometer")
accelerometer.SetBody(ground_body)
sensor_mgr.AddSensor(accelerometer)

# Gyroscope
gyroscope = sens.ChGyroscopeSensor()
gyroscope.SetName("gyroscope")
gyroscope.SetBody(ground_body)
sensor_mgr.AddSensor(gyroscope)

# Magnetometer
magnetometer = sens.ChMagnetometerSensor()
magnetometer.SetName("magnetometer")
magnetometer.SetBody(ground_body)
sensor_mgr.AddSensor(magnetometer)

# Initialize ROS manager and register sensors
ros_mgr = sens.ChROSManager()
ros_mgr.SetSensorManager(sensor_mgr)
ros_mgr.RegisterSensor(camera, "/camera/data")
ros_mgr.RegisterSensor(lidar, "/lidar/scan")
ros_mgr.RegisterSensor(gps, "/gps/fix")
ros_mgr.RegisterSensor(accelerometer, "/imu/accel")
ros_mgr.RegisterSensor(gyroscope, "/imu/gyro")
ros_mgr.RegisterSensor(magnetometer, "/imu/mag")
ros_mgr.InitNode("chrono_sensor_publisher")

# Setup visualization
vis = cr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Sensor Simulation")
vis.SetSymbolscale(0.01)
vis.SetShadows(True)
vis.SetLightIntensity(0.8)
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
vis.SetRenderMode(cr.RENDER_NORMAL)
sys.SetVisualSystem(vis)
vis.AttachBodyFrame(ground_body)
vis.Initialize()

# Simulation parameters
time_step = 0.001
real_time_factor = 1.0
max_time = 10.0

# Simulation loop
while sys.GetChTime() < max_time:
    # Update sensors
    sensor_mgr.Update()
    
    # Apply force to ground body to induce motion
    ground_body.AddForce(chrono.ChVectorD(100, 0, 0))  # Example force
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Publish sensor data via ROS
    ros_mgr.Update()
    
    # Visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time
    chrono.ChSleepTime(sys.GetChTimeStep() * real_time_factor)