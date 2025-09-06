import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np
import os
import time




system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))





ground = chrono.ChBodyEasyBox(10, 10, 0.2, 1000, True, True)
ground.SetName("MovingGround")
ground.SetPos(chrono.ChVector3d(0, 0, -1))
system.Add(ground)


motion_func = chrono.ChFunctionSine(0.5, 0.5)  
motion = chrono.ChLinkMotorLinPosition()
motion.Initialize(ground, system.GetChBody(0), 
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motion.SetPositionFunction(motion_func)
system.Add(motion)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))
mesh_asset = chrono.ChVisualShapeTriangleMesh()
mesh_asset.SetMesh(mesh)
mesh_asset.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
ground.AddVisualShape(mesh_asset)




manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 100)


cam_offset = chrono.ChFramed(chrono.ChVector3d(-2, 0, 1), 
                            chrono.QuatFromAngleY(np.pi/2))
camera = sens.ChCameraSensor(
    ground,             
    30,                 
    cam_offset,         
    1280,               
    720,                
    np.deg2rad(75)      
)
camera.SetName("RGB_Camera")
camera.PushFilter(sens.ChFilterRGBA8())
manager.AddSensor(camera)


lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5))
lidar = sens.ChLidarSensor(
    ground,             
    10,                 
    lidar_offset,       
    1000,               
    50,                 
    np.deg2rad(30),     
    np.deg2rad(15),     
    0.1,                
    100.0               
)
lidar.SetName("Scanning_Lidar")
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
manager.AddSensor(lidar)


imu_offset = chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0.3))
accel = sens.ChAccelerometerSensor(ground, 100, imu_offset)
gyro = sens.ChGyroscopeSensor(ground, 100, imu_offset)
mag = sens.ChMagnetometerSensor(ground, 100, imu_offset)

accel.SetName("IMU_Accelerometer")
gyro.SetName("IMU_Gyroscope")
mag.SetName("IMU_Magnetometer")


accel.PushFilter(sens.ChFilterAccelNoiseNormal(0.001, 0.01))
gyro.PushFilter(sens.ChFilterGyroNoiseNormal(0.001, 0.001))
mag.PushFilter(sens.ChFilterMagnetNoiseNormal(0.02, 0.02))

manager.AddSensor(accel)
manager.AddSensor(gyro)
manager.AddSensor(mag)


gps_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5))
gps = sens.ChGPSSensor(
    ground,             
    10,                 
    gps_offset,         
    chrono.ChVector3d(0, 0, 0),  
    True                
)
gps.SetName("Positioning_GPS")
manager.AddSensor(gps)




ros_manager = sens.ChROSParallelManager()
ros_manager.RegisterSensor(camera, "/pychrono/camera")
ros_manager.RegisterSensor(lidar, "/pychrono/lidar")
ros_manager.RegisterSensor(accel, "/pychrono/imu/accel")
ros_manager.RegisterSensor(gyro, "/pychrono/imu/gyro")
ros_manager.RegisterSensor(mag, "/pychrono/imu/mag")
ros_manager.RegisterSensor(gps, "/pychrono/gps")




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Sensor Demo")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(3, 3, 3))
vis.AddTypicalLights()
vis.AddSkyBox()




time_step = 0.001
real_time_factor = 1.0
last_time = time.time()

while vis.Run():
    
    current_time = time.time()
    elapsed = current_time - last_time
    last_time = current_time
    
    
    manager.Update()
    ros_manager.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    
    computation_time = time.time() - current_time
    sleep_time = time_step/real_time_factor - computation_time
    if sleep_time > 0:
        time.sleep(sleep_time)