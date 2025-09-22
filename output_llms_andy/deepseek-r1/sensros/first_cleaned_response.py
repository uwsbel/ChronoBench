import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import math




chrono.SetChronoDataPath("")  
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))




ground = chrono.ChBodyEasyBox(10, 10, 0.2, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(False)  
system.Add(ground)


box_mesh = chrono.ChBoxShape()
box_mesh.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
box_mesh.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 0)
ground.AddVisualShape(box_mesh)


motion_func = chrono.ChFunction_Sine(0, 0.5, 0.2)  
ground.SetPosDtFunction(motion_func)




sensor_manager = sens.ChSensorManager(system)
sensor_offset = chrono.ChVectorD(0, 0, 0.1)  


cam = sens.ChCameraSensor(
    ground,                         
    30,                             
    chrono.ChFrameD(sensor_offset), 
    1280,                           
    720,                            
    chrono.CH_C_PI / 3              
)
cam.SetName("Camera")
cam.PushFilter(sens.ChFilterRGBA8Access())
sensor_manager.AddSensor(cam)


lidar = sens.ChLidarSensor(
    ground,                         
    15,                             
    chrono.ChFrameD(sensor_offset), 
    1000,                           
    chrono.CH_C_PI / 3,             
    chrono.CH_C_PI / 6,             
    0.1,                            
    50.0                            
)
lidar.SetName("Lidar")
lidar.PushFilter(sens.ChFilterDIAccess())
sensor_manager.AddSensor(lidar)


imu_offset = chrono.ChFrameD(sensor_offset)
imu_noise = sens.ChNoiseNormal(chrono.ChVectorD(0.001), chrono.ChVectorD(0.001))


accel = sens.ChAccelerometerSensor(
    ground, 10, imu_offset, imu_noise)
accel.SetName("Accelerometer")
sensor_manager.AddSensor(accel)


gyro = sens.ChGyroscopeSensor(
    ground, 10, imu_offset, imu_noise)
gyro.SetName("Gyroscope")
sensor_manager.AddSensor(gyro)


mag = sens.ChMagnetometerSensor(
    ground, 10, imu_offset, imu_noise)
mag.SetName("Magnetometer")
mag.SetMagneticField(chrono.ChVectorD(0, 0, -1e-5))  
sensor_manager.AddSensor(mag)


gps = sens.ChGPSSensor(
    ground, 10, imu_offset, chrono.ChVectorD(0,0,0), imu_noise)
gps.SetName("GPS")
sensor_manager.AddSensor(gps)




ros_manager = sens.ChROSPublisher()
ros_manager.RegisterPublisher(cam, "/sensors/camera")
ros_manager.RegisterPublisher(lidar, "/sensors/lidar")
ros_manager.RegisterPublisher(accel, "/sensors/imu/accel")
ros_manager.RegisterPublisher(gyro, "/sensors/imu/gyro")
ros_manager.RegisterPublisher(mag, "/sensors/imu/mag")
ros_manager.RegisterPublisher(gps, "/sensors/gps")




vis = irr.ChIrrApp(system, "Sensor Demo", irr.dimension2du(1280, 720))
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AttachSystem(system)




time_step = 0.001
end_time = 20.0
current_time = 0.0

while vis.Run() and current_time < end_time:
    
    sensor_manager.Update()
    
    
    ros_manager.Process()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    
    chrono.ChRealtimeStepTimer.Spin(time_step)
    current_time += time_step