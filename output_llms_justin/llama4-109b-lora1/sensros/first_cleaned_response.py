import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono_ros as ros
import math
import numpy as np


chrono.ChSystemNSC.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


mesh = veh.GetDataFile("mesh/cubicle.obj")
ground_shape = chrono.ChVisualShapeTriangleMesh(
    mesh_file=mesh,
    color=chrono.ChColor(1, 1, 1),
    add_collision=True,
    visible=True,
)
ground.AddVisualShape(ground_shape)


sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetName("SensorManager")
sensor_manager.Initialize(ground)


camera = sens.ChCameraSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
camera.SetName("Camera")
camera.SetUpdateRate(1)
camera.SetCollectionMode(sens.Sensor.CollectMode_Once)
sensor_manager.AddSensor(camera)

lidar = sens.ChLidarSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
lidar.SetName("Lidar")
lidar.SetUpdateRate(1)
lidar.SetCollectionMode(sens.Sensor.CollectMode_Once)
lidar.SetHorizontalResolution(100)
lidar.SetVerticalResolution(20)
lidar.SetMinFov(0.01)
lidar.SetMaxFov(100)
sensor_manager.AddSensor(lidar)

gps = sens.ChGPSSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
gps.SetName("GPS")
gps.SetUpdateRate(1)
gps.SetCollectionMode(sens.Sensor.CollectMode_Once)
sensor_manager.AddSensor(gps)

accelerometer = sens.ChAccelerometerSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
accelerometer.SetName("Accelerometer")
accelerometer.SetUpdateRate(1)
accelerometer.SetCollectionMode(sens.Sensor.CollectMode_Once)
sensor_manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
gyroscope.SetName("Gyroscope")
gyroscope.SetUpdateRate(1)
gyroscope.SetCollectionMode(sens.Sensor.CollectMode_Once)
sensor_manager.AddSensor(gyroscope)

magnetometer = sens.ChMagnetometerSensor(ground, 1000, chrono.ChVector3d(0, 0, 1), 0, 30, 0)
magnetometer.SetName("Magnetometer")
magnetometer.SetUpdateRate(1)
magnetometer.SetCollectionMode(sens.Sensor.CollectMode_Once)
sensor_manager.AddSensor(magnetometer)


ros_manager = ros.ChROSManager()
ros_manager.Initialize(ground, "ground", "sensor_data")
ros_manager.RegisterSensor(camera, "camera/color/image_raw")
ros_manager.RegisterSensor(lidar, "scan")
ros_manager.RegisterSensor(gps, "gps/fix")
ros_manager.RegisterSensor(accelerometer, "accelerometer")
ros_manager.RegisterSensor(gyroscope, "gyroscope")
ros_manager.RegisterSensor(magnetometer, "magnetometer")


step_size = 1e-3
real_time_sim = chrono.ChRealtimeStepSim()
real_time_sim.SetStepSize(step_size)
real_time_sim.SetTimeStep(step_size)

while True:
    sensor_manager.Update()
    system.DoStepDynamics(step_size)
    real_time_sim.DoStepDynamics(step_size)
    irr.ChVisualSystemIrrlicht.ShowStats()
    irr.ChVisualSystemIrrlicht.UpdateCamera()
    if real_time_sim.GetStopRequest():
        break