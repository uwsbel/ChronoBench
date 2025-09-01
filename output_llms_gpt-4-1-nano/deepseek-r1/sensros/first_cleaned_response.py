import pychrono as chrono
import pychrono.ros as chrono_ros
import pychrono.sensor as sens
import math


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))


ground = chrono.ChBody()
ground.SetIdentifier(0)
ground.SetBodyFixed(False)
sys.Add(ground)


mesh = chrono.ChObjFileShape()
mesh.SetFilename(chrono.GetChronoDataFile("models/cube.obj"))
mesh.SetScale(chrono.ChVectorD(2, 0.1, 2))
ground.AddVisualShape(mesh)


motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(ground, chrono.ChBodyFrame(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.AddLink(motor)
motion_func = chrono.ChFunction_Sine(0.5, 0.5)  
motor.SetPositionFunction(motion_func)


sensor_manager = sens.ChSensorManager(sys)
sensor_manager.scene.AddPointLight(chrono.ChVectorD(0, 10, 0), chrono.ChColor(1, 1, 1), 1000)


update_rate = 30  
lag = 0.0
exposure_time = 0.0


camera_offset = chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), 
                               chrono.Q_from_AngAxis(-chrono.CH_C_PI/2, chrono.ChVectorD(1, 0, 0)))
camera = sens.ChCameraSensor(
    ground, update_rate, camera_offset,
    1280, 720, 1.396,  
    sens.CameraLensModelType_PINHOLE
)
camera.SetName("Camera")
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(chrono_ros.ChFilterROS2Topic("camera", "rgb"))
sensor_manager.AddSensor(camera)


lidar_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0))
lidar = sens.ChLidarSensor(
    ground, 10, lidar_offset,
    1000, 100,  
    2*chrono.CH_C_PI, chrono.CH_C_PI/6, 100.0  
)
lidar.SetName("Lidar")
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(chrono_ros.ChFilterROS2Topic("lidar", "points"))
sensor_manager.AddSensor(lidar)


gps_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0))
gps = sens.ChGPSSensor(
    ground, 10, gps_offset,
    chrono.ChVectorD(0, 0, 0)  
)
gps.PushFilter(chrono_ros.ChFilterROS2Topic("gps", "nav"))
sensor_manager.AddSensor(gps)


imu_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0.1, 0))


accel = sens.ChAccelerometerSensor(ground, 100, imu_offset)
accel.PushFilter(chrono_ros.ChFilterROS2Topic("accelerometer", "imu"))
sensor_manager.AddSensor(accel)


gyro = sens.ChGyroscopeSensor(ground, 100, imu_offset)
gyro.PushFilter(chrono_ros.ChFilterROS2Topic("gyroscope", "imu"))
sensor_manager.AddSensor(gyro)


mag = sens.ChMagnetometerSensor(ground, 100, imu_offset)
mag.SetMagneticFieldFunction(lambda pos: chrono.ChVectorD(0, 0, 1e-5))
mag.PushFilter(chrono_ros.ChFilterROS2Topic("magnetometer", "mag"))
sensor_manager.AddSensor(mag)


chrono_ros.ChROS2.Initialize()
node = chrono_ros.ChROS2Node("chrono_sensor_node")
sensor_manager.GetROS2Publisher().Register(node)


time_step = 0.001
duration = 10  
realtime = True
timer = chrono.ChRealtimeStepTimer()

while sys.GetChTime() < duration:
    
    sensor_manager.Update()
    
    
    sys.DoStepDynamics(time_step)
    
    
    node.SpinOnce()
    
    
    if realtime:
        timer.Spin(time_step)