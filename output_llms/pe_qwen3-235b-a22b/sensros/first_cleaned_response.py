import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetMass(1000)
ground.SetInertiaXX(chrono.ChVector3d(1e3, 1e3, 1e3))
ground_shape = chrono.ChVisualShapeBox(10, 1, 10)
ground_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_shape)
sys.Add(ground)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(ground, chrono.ChBody(), chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(prismatic)

motor = chrono.ChLinkMotorLinearSpeed()
motor.Initialize(ground, chrono.ChBody(), chrono.ChCoordsysD())
func = chrono.ChFunction_Sine(0, 1, 1.0)  
motor.SetFunction(func)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Sensors on Moving Ground')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


manager = sens.ChSensorManager(sys)
manager.SetTimeStep(1e-3)  



camera = sens.ChCameraSensor(
    ground,
    60,  
    chrono.ChFrameD(chrono.ChVector3d(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    640, 480,  
    0.01  
)
camera.SetName("camera")
manager.AddSensor(camera)


lidar = sens.ChLidarSensor(
    ground,
    10,  
    chrono.ChFrameD(chrono.ChVector3d(0, 1, 0)),
    100, 360,  
    1.0,  
    0.1,  
    0.01  
)
lidar.SetName("lidar")
manager.AddSensor(lidar)


gps = sens.ChGPSSensor(
    ground,
    1,  
    chrono.ChFrameD(),
    chrono.ChVector3d(0, 0, 0),  
    1.0  
)
gps.SetName("gps")
manager.AddSensor(gps)


accelerometer = sens.ChAccelerometerSensor(
    ground,
    100,  
    chrono.ChFrameD()
)
accelerometer.SetName("accelerometer")
manager.AddSensor(accelerometer)


gyro = sens.ChGyroscopeSensor(
    ground,
    100,  
    chrono.ChFrameD()
)
gyro.SetName("gyro")
manager.AddSensor(gyro)


magnetometer = sens.ChMagnetometerSensor(
    ground,
    100,  
    chrono.ChFrameD()
)
magnetometer.SetName("magnetometer")
manager.AddSensor(magnetometer)


ros_manager = chros.ChROSPublisherManager()
ros_manager.RegisterSensor(camera, "/camera/image")
ros_manager.RegisterSensor(lidar, "/lidar/scan")
ros_manager.RegisterSensor(gps, "/gps/data")
ros_manager.RegisterSensor(accelerometer, "/imu/accel")
ros_manager.RegisterSensor(gyro, "/imu/gyro")
ros_manager.RegisterSensor(magnetometer, "/imu/mag")


step_size = 1e-3
time = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    
    manager.Update()
    
    
    sys.DoStepDynamics(step_size)
    time += step_size
    
    
    ros_manager.Update(time)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)