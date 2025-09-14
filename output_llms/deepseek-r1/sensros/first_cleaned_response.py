import os
import math
import chrono
import irrlicht
import chrono.sensor as csensor
from chrono.sensor.ros import ChROSTopicManager


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


ground_body = chrono.ChBodyEasyBox(2, 0.2, 2, 1000, True, True)
ground_body.SetBodyFixed(False)
system.Add(ground_body)


motor = chrono.ChLinkMotorLinearPosition()
motor.Initialize(ground_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetPositionFunction(chrono.ChFunction_Sine(0.5, 0.5))  
system.Add(motor)


sensor_manager = csensor.ChSensorManager(system)
sensor_manager.scene.AddAmbientLight(chrono.ChVectorF(100, 100, 100), 1.0)


parent_body = ground_body
sensor_offset = chrono.ChVectorD(0, 0.2, 0)


camera = csensor.ChCameraSensor(
    parent_body,
    30,
    chrono.ChFrameD(sensor_offset + chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
    1280,
    720,
    chrono.CH_C_PI / 3
)
camera.PushFilter(csensor.ChFilterRGBA8())
sensor_manager.AddSensor(camera)


lidar = csensor.ChLidarSensor(
    parent_body,
    10,
    chrono.ChFrameD(sensor_offset, chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
    1024,
    64,
    chrono.CH_C_PI / 2,
    chrono.CH_C_PI / 6,
    0.1,
    100.0
)
lidar.PushFilter(csensor.ChFilterDIAccess())
lidar.PushFilter(csensor.ChFilterPCfromDepth())
sensor_manager.AddSensor(lidar)


gps = csensor.ChGPSSensor(
    parent_body,
    50,
    chrono.ChFrameD(sensor_offset),
    chrono.ChVectorD(0, 0, 0),
    csensor.GPSNoiseNormal(0.1, 0.2)
)
sensor_manager.AddSensor(gps)

accel = csensor.ChAccelerometerSensor(
    parent_body,
    100,
    chrono.ChFrameD(sensor_offset),
    csensor.AccelNoiseNormal(0.01, 0.02)
)
sensor_manager.AddSensor(accel)

gyro = csensor.ChGyroscopeSensor(
    parent_body,
    100,
    chrono.ChFrameD(sensor_offset),
    csensor.GyroNoiseNormal(0.001, 0.002)
)
sensor_manager.AddSensor(gyro)

mag = csensor.ChMagnetometerSensor(
    parent_body,
    50,
    chrono.ChFrameD(sensor_offset),
    chrono.ChVectorD(0, 0, 1),
    csensor.MagNoiseNormal(0.001, 0.002)
)
sensor_manager.AddSensor(mag)


ros_manager = ChROSTopicManager()
ros_manager.RegisterHandler("chrono_ros")


ros_manager.AddSensor(camera, "/camera/image")
ros_manager.AddSensor(lidar, "/lidar/points")
ros_manager.AddSensor(gps, "/gps/data")
ros_manager.AddSensor(accel, "/imu/accel")
ros_manager.AddSensor(gyro, "/imu/gyro")
ros_manager.AddSensor(mag, "/imu/mag")


viz = irrlicht.ChIrrApp(system, "Sensor Demo", 800, 600)
viz.AddTypicalLights()
viz.AddTypicalCamera(irrlicht.vector3df(2, 2, 2))
viz.AssetBindAll()
viz.AssetUpdateAll()


time_step = 0.001
realtime_timer = chrono.ChRealtimeStepTimer()

while viz.GetDevice().run():
    viz.BeginScene()
    viz.DrawAll()
    viz.EndScene()
    
    
    system.DoStepDynamics(time_step)
    
    
    sensor_manager.Update()
    
    
    ros_manager.Update(system.GetChTime())
    
    
    realtime_timer.Spin(time_step)