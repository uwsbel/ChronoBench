import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensors as sens
import rospy
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu, MagneticField


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', 'data'))


rospy.init_node('pychrono_sensor_publisher', anonymous=True)


sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Sensors Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 10, 0.1))
ground.AddVisualShape(ground_shape)
sys.Add(ground)


sensor_manager = sens.ChSensorManager(sys)
ros_manager = sens.ChRosManager(sys)


camera = sens.ChCamera()
camera.SetChassisBody(ground)
camera.SetFieldOfView(90)
sensor_manager.Add(camera)
ros_manager.RegisterSensor('camera', '/camera/image_raw', Image)

lidar = sens.ChLidar()
lidar.SetChassisBody(ground)
lidar.SetRange(50)
sensor_manager.Add(lidar)
ros_manager.RegisterSensor('lidar', '/lidar/points', PointCloud2)

gps = sens.ChGPS()
gps.SetChassisBody(ground)
sensor_manager.Add(gps)
ros_manager.RegisterSensor('gps', '/gps/fix', NavSatFix)

accelerometer = sens.ChAccelerometer()
accelerometer.SetChassisBody(ground)
sensor_manager.Add(accelerometer)
ros_manager.RegisterSensor('accelerometer', '/imu/data', Imu)

gyroscope = sens.ChGyroscope()
gyroscope.SetChassisBody(ground)
sensor_manager.Add(gyroscope)
ros_manager.RegisterSensor('gyroscope', '/imu/data', Imu)

magnetometer = sens.ChMagnetometer()
magnetometer.SetChassisBody(ground)
sensor_manager.Add(magnetometer)
ros_manager.RegisterSensor('magnetometer', '/magnetometer/data', MagneticField)


while not rospy.is_shutdown():
    
    sensor_manager.Update()
    ros_manager.PublishAll()

    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    vis.WaitForFrameEnd()


rospy.signal_shutdown('Simulation ended')