import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import pychrono.ros as ros

# --- Simulation Setup ---
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --- Visualization Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Sensor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6))
vis.AddTypicalLights()

# --- Ground Body ---
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVector3d(0, -0.5, 0))
ground.SetShapeFromMesh(chrono.GetChronoDataFile('ground.obj'))  # Replace with your ground mesh
ground.CreateVisualShape(chrono.ChVisualShapeMesh(chrono.GetChronoDataFile('ground.obj')))
sys.Add(ground)

# --- Sensor Manager ---
sensor_manager = sensor.ChSensorManager()

# --- Sensors ---
# Camera
camera = sensor.ChCameraSensor()
camera.SetResolution(640, 480)
camera.SetFOV(60)
camera.AttachBody(ground)
sensor_manager.AddSensor(camera)

# Lidar
lidar = sensor.ChLidarSensor()
lidar.SetRange(10)
lidar.SetResolution(180)
lidar.AttachBody(ground)
sensor_manager.AddSensor(lidar)

# GPS
gps = sensor.ChGPSSensor()
gps.AttachBody(ground)
sensor_manager.AddSensor(gps)

# Accelerometer
accelerometer = sensor.ChAccelerometerSensor()
accelerometer.AttachBody(ground)
sensor_manager.AddSensor(accelerometer)

# Gyroscope
gyroscope = sensor.ChGyroscopeSensor()
gyroscope.AttachBody(ground)
sensor_manager.AddSensor(gyroscope)

# Magnetometer
magnetometer = sensor.ChMagnetometerSensor()
magnetometer.AttachBody(ground)
sensor_manager.AddSensor(magnetometer)

# --- ROS Manager ---
ros_manager = ros.ChRosManager()
ros_manager.Initialize(sys)

# Register sensors with ROS
ros_manager.RegisterSensor(camera, 'camera_data')
ros_manager.RegisterSensor(lidar, 'lidar_data')
ros_manager.RegisterSensor(gps, 'gps_data')
ros_manager.RegisterSensor(accelerometer, 'accelerometer_data')
ros_manager.RegisterSensor(gyroscope, 'gyroscope_data')
ros_manager.RegisterSensor(magnetometer, 'magnetometer_data')

# --- Ground Movement ---
ground_speed = 0.5
ground_direction = chrono.ChVector3d(1, 0, 0)

# --- Simulation Loop ---
while vis.Run():
    # Update sensors
    sensor_manager.Update(sys.GetChTime())

    # Move the ground
    ground_movement = ground_speed * ground_direction * sys.GetStep()
    ground.SetPos(ground.GetPos() + ground_movement)

    # Advance the simulation
    sys.DoStepDynamics(0.01)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Publish ROS data
    ros_manager.PublishData()