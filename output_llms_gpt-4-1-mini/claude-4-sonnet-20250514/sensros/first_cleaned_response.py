import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros
import numpy as np
import math
import time

def main():
    print("Starting PyChrono Multi-Sensor Simulation...")
    
    
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    
    
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(False)  
    ground.SetPos(chrono.ChVectorD(0, 0, 1))
    ground.SetMass(1000)
    ground.SetInertiaXX(chrono.ChVectorD(100, 100, 100))
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 2, 0.1)
    ground.AddShape(ground_shape, chrono.ChFrameD())
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.1)
    ground.GetShapeInstance(0).SetMaterial(material)
    
    system.Add(ground)
    
    
    
    
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(10, 0, 0))
    
    
    mesh_shape = chrono.ChBoxShape()
    mesh_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 1, 1)
    mesh_body.AddShape(mesh_shape, chrono.ChFrameD())
    
    
    mesh_visual = chrono.ChVisualShapeBox(2, 2, 2)
    mesh_visual.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    mesh_body.AddVisualShape(mesh_visual)
    
    system.Add(mesh_body)
    
    
    
    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChColor(1, 1, 1), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(0, -100, 0), chrono.ChColor(1, 1, 1), 500.0)
    
    
    
    
    
    
    camera = sens.ChCameraSensor(
        ground,  
        30,      
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
        1280,    
        720,     
        chrono.CH_C_PI / 3  
    )
    camera.SetName("camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    
    lidar = sens.ChLidarSensor(
        ground,  
        10,      
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5), chrono.QUNIT),
        1800,    
        16,      
        chrono.CH_C_PI,     
        chrono.CH_C_PI/12,  
        -chrono.CH_C_PI/6,  
        100.0    
    )
    lidar.SetName("lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)
    
    
    gps = sens.ChGPSSensor(
        ground,  
        10,      
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.QUNIT),
        chrono.ChVectorD(0, 0, 0),  
        sens.ChGPSSensor.GPS_REFERENCE.WGS84
    )
    gps.SetName("gps")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)
    
    
    accelerometer = sens.ChAccelerometerSensor(
        ground,  
        100,     
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
    )
    accelerometer.SetName("accelerometer")
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)
    
    
    gyroscope = sens.ChGyroscopeSensor(
        ground,  
        100,     
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
    )
    gyroscope.SetName("gyroscope")
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)
    
    
    magnetometer = sens.ChMagnetometerSensor(
        ground,  
        100,     
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT),
        sens.ChMagnetometerSensor.MagneticFieldModel.DIPOLE_MODEL
    )
    magnetometer.SetName("magnetometer")
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magnetometer)
    
    
    
    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSCameraHandler("~/camera/image", camera))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler("~/lidar/points", lidar))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler("~/gps/data", gps))
    ros_manager.RegisterHandler(chros.ChROSAccelerometerHandler("~/imu/accel", accelerometer))
    ros_manager.RegisterHandler(chros.ChROSGyroscopeHandler("~/imu/gyro", gyroscope))
    ros_manager.RegisterHandler(chros.ChROSMagnetometerHandler("~/imu/mag", magnetometer))
    
    
    ros_manager.Initialize()
    
    
    
    
    def update_ground_motion(time):
        
        
        radius = 5.0
        angular_freq = 0.5
        
        x = radius * math.cos(angular_freq * time)
        y = radius * math.sin(angular_freq * time)
        z = 1.0 + 0.5 * math.sin(2 * angular_freq * time)
        
        
        ground.SetPos(chrono.ChVectorD(x, y, z))
        
        
        vx = -radius * angular_freq * math.sin(angular_freq * time)
        vy = radius * angular_freq * math.cos(angular_freq * time)
        vz = angular_freq * math.cos(2 * angular_freq * time)
        ground.SetPos_dt(chrono.ChVectorD(vx, vy, vz))
        
        
        angle = angular_freq * time
        ground.SetRot(chrono.Q_from_AngZ(angle))
        ground.SetWvel_par(chrono.ChVectorD(0, 0, angular_freq))
    
    
    
    
    step_size = 0.01  
    simulation_time = 0
    max_simulation_time = 30.0  
    
    
    real_time_timer = chrono.ChRealtimeStepTimer()
    
    print("Starting simulation loop...")
    print(f"Simulation will run for {max_simulation_time} seconds")
    print("ROS topics:")
    print("  - Camera: ~/camera/image")
    print("  - Lidar: ~/lidar/points")
    print("  - GPS: ~/gps/data")
    print("  - Accelerometer: ~/imu/accel")
    print("  - Gyroscope: ~/imu/gyro")
    print("  - Magnetometer: ~/imu/mag")
    
    
    
    
    while simulation_time < max_simulation_time:
        
        update_ground_motion(simulation_time)
        
        
        manager.Update()
        
        
        system.DoStepDynamics(step_size)
        
        
        if not ros_manager.Update(simulation_time, step_size):
            break
        
        
        real_time_timer.Spin(step_size)
        
        
        simulation_time += step_size
        
        
        if int(simulation_time) % 5 == 0 and simulation_time % 5 < step_size:
            print(f"Simulation time: {simulation_time:.1f}s / {max_simulation_time}s")
            pos = ground.GetPos()
            print(f"Ground position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    
    print("Simulation completed successfully!")
    print("Sensor data has been published to ROS topics throughout the simulation.")

if __name__ == "__main__":
    main()