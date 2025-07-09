import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('sensor/textured_cylinder.obj'), True, True)
mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetBodyFixed(False)
body_shape = chrono.ChVisualShape()
body_shape.SetMesh(mesh)
body.AddVisualShape(body_shape)
sys.Add(body)


ground_body = chrono.ChBody()
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetBodyFixed(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 1, 10))
ground_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
ground_body.AddVisualShape(ground_shape)
sys.Add(ground_body)


manager = sens.ChSensorManager(sys)


camera = sens.ChCameraSensor(
    ground_body, 
    10, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2)), 
    640, 
    480, 
    chrono.CH_PI / 4
)
camera.SetName("Camera Sensor")
manager.AddSensor(camera)


lidar = sens.ChLidarSensor(
    ground_body, 
    10, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2)), 
    1000, 
    chrono.CH_PI / 2, 
    chrono.CH_PI / 4, 
    100
)
lidar.SetName("Lidar Sensor")
manager.AddSensor(lidar)


gps = sens.ChGPSSensor(
    ground_body, 
    10, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2)), 
    chrono.ChVector3d(0, 0, 0)
)
gps.SetName("GPS Sensor")
manager.AddSensor(gps)


accelerometer = sens.ChAccelerometerSensor(
    ground_body, 
    100, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2))
)
accelerometer.SetName("Accelerometer Sensor")
manager.AddSensor(accelerometer)


gyroscope = sens.ChGyroscopeSensor(
    ground_body, 
    100, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2))
)
gyroscope.SetName("Gyroscope Sensor")
manager.AddSensor(gyroscope)


magnetometer = sens.ChMagnetometerSensor(
    ground_body, 
    100, 
    chrono.ChFrame(chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleX(np.pi / 2)), 
    chrono.ChVector3d(0, 0, 0)
)
magnetometer.SetName("Magnetometer Sensor")
manager.AddSensor(magnetometer)


ros_manager = sens.ChROSPythonManager("sensor_node")
ros_manager.RegisterSensor(camera, "camera_topic", "sensor_msgs/msg/Image")
ros_manager.RegisterSensor(lidar, "lidar_topic", "sensor_msgs/msg/PointCloud2")
ros_manager.RegisterSensor(gps, "gps_topic", "sensor_msgs/msg/NavSatFix")
ros_manager.RegisterSensor(accelerometer, "accelerometer_topic", "sensor_msgs/msg/Imu")
ros_manager.RegisterSensor(gyroscope, "gyroscope_topic", "sensor_msgs/msg/Imu")
ros_manager.RegisterSensor(magnetometer, "magnetometer_topic", "sensor_msgs/msg/MagneticField")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Sensor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    manager.Update()

    
    sys.DoStepDynamics(0.01)

    
    ros_manager.Update()