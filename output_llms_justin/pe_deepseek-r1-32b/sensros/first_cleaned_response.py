import math
import numpy as np
import pychrono as chrono
import pychrono.robot as rob
import pychrono.sensor as sensor
import pychrono.ros as ros


ros_manager = ros.ChROSManager()
ros_manager.Initialize()


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetMass(0)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(False)  
system.Add(ground)


mesh = chrono.ChMesh()
mesh.SetName("ground_mesh")
mesh.SetMaterialSurface(chrono.ChVisualMaterial())
mesh.SetScale(chrono.ChVectorD(1, 1, 1))
ground.AddVisualShape(mesh)


sensor_manager = sensor.ChSensorManager(system)



camera = sensor.ChCameraSensor(ground, chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))
ros_manager.RegisterSensor("camera", camera)


lidar = sensor.ChLidarSensor(ground, 360, 10, 0.1, 10)
ros_manager.RegisterSensor("lidar", lidar)


gps = sensor.ChGPSSensor(ground)
ros_manager.RegisterSensor("gps", gps)


accelerometer = sensor.ChAccelerometerSensor(ground)
ros_manager.RegisterSensor("accelerometer", accelerometer)


gyroscope = sensor.ChGyroscopeSensor(ground)
ros_manager.RegisterSensor("gyroscope", gyroscope)


magnetometer = sensor.ChMagnetometerSensor(ground)
ros_manager.RegisterSensor("magnetometer", magnetometer)


sensor_manager.AddSensor(camera)
sensor_manager.AddSensor(lidar)
sensor_manager.AddSensor(gps)
sensor_manager.AddSensor(accelerometer)
sensor_manager.AddSensor(gyroscope)
sensor_manager.AddSensor(magnetometer)


ros_manager.InitializePublishers()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensor Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddLight(chrono.ChVectorD(0, 10, 0), chrono.ChColor(1, 1, 1))


time_step = 0.001
real_time_factor = 1.0
simulation_time = 0


while vis.Run():
    
    sensor_manager.Update()
    
    
    ros_manager.PublishSensorData()
    
    
    system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    key = vis.GetKeyboard()
    if key == ord('w'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0, 0, 0.1))
    elif key == ord('s'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0, 0, -0.1))
    elif key == ord('a'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(-0.1, 0, 0))
    elif key == ord('d'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0.1, 0, 0))
    
    
    if vis.GetKeyboard() == ord('q'):
        break
    
    
    simulation_time += time_step


ros_manager.Shutdown()
vis.Close()