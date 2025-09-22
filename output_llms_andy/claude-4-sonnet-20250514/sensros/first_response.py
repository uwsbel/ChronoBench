#!/usr/bin/env python3

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros
import numpy as np
import math
import time

def main():
    print("Starting PyChrono Multi-Sensor Simulation...")
    
    # -----------------------------
    # Initialize the Chrono system
    # -----------------------------
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Set solver parameters
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    
    # -----------------------------
    # Create ground body
    # -----------------------------
    ground = chrono.ChBody()
    ground.SetBodyFixed(False)  # Allow movement
    ground.SetPos(chrono.ChVectorD(0, 0, 1))
    ground.SetMass(1000)
    ground.SetInertiaXX(chrono.ChVectorD(100, 100, 100))
    
    # Add collision shape to ground
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 2, 0.1)
    ground.AddShape(ground_shape, chrono.ChFrameD())
    
    # Set material properties
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.4)
    material.SetRestitution(0.1)
    ground.GetShapeInstance(0).SetMaterial(material)
    
    system.Add(ground)
    
    # -----------------------------
    # Add mesh object for visualization
    # -----------------------------
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(10, 0, 0))
    
    # Create a simple box mesh for visualization
    mesh_shape = chrono.ChBoxShape()
    mesh_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 1, 1)
    mesh_body.AddShape(mesh_shape, chrono.ChFrameD())
    
    # Add visual mesh
    mesh_visual = chrono.ChVisualShapeBox(2, 2, 2)
    mesh_visual.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    mesh_body.AddVisualShape(mesh_visual)
    
    system.Add(mesh_body)
    
    # -----------------------------
    # Initialize sensor manager
    # -----------------------------
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChColor(1, 1, 1), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(0, -100, 0), chrono.ChColor(1, 1, 1), 500.0)
    
    # -----------------------------
    # Create and configure sensors
    # -----------------------------
    
    # 1. Camera sensor
    camera = sens.ChCameraSensor(
        ground,  # Attach to ground body
        30,      # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
        1280,    # Image width
        720,     # Image height
        chrono.CH_C_PI / 3  # Field of view
    )
    camera.SetName("camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    # 2. Lidar sensor
    lidar = sens.ChLidarSensor(
        ground,  # Attach to ground body
        10,      # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5), chrono.QUNIT),
        1800,    # Horizontal samples
        16,      # Vertical channels
        chrono.CH_C_PI,     # Horizontal field of view
        chrono.CH_C_PI/12,  # Max vertical angle
        -chrono.CH_C_PI/6,  # Min vertical angle
        100.0    # Max range
    )
    lidar.SetName("lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)
    
    # 3. GPS sensor
    gps = sens.ChGPSSensor(
        ground,  # Attach to ground body
        10,      # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.QUNIT),
        chrono.ChVectorD(0, 0, 0),  # GPS reference point
        sens.ChGPSSensor.GPS_REFERENCE.WGS84
    )
    gps.SetName("gps")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)
    
    # 4. Accelerometer
    accelerometer = sens.ChAccelerometerSensor(
        ground,  # Attach to ground body
        100,     # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
    )
    accelerometer.SetName("accelerometer")
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)
    
    # 5. Gyroscope
    gyroscope = sens.ChGyroscopeSensor(
        ground,  # Attach to ground body
        100,     # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
    )
    gyroscope.SetName("gyroscope")
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)
    
    # 6. Magnetometer
    magnetometer = sens.ChMagnetometerSensor(
        ground,  # Attach to ground body
        100,     # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT),
        sens.ChMagnetometerSensor.MagneticFieldModel.DIPOLE_MODEL
    )
    magnetometer.SetName("magnetometer")
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magnetometer)
    
    # -----------------------------
    # Initialize ROS manager
    # -----------------------------
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSCameraHandler("~/camera/image", camera))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler("~/lidar/points", lidar))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler("~/gps/data", gps))
    ros_manager.RegisterHandler(chros.ChROSAccelerometerHandler("~/imu/accel", accelerometer))
    ros_manager.RegisterHandler(chros.ChROSGyroscopeHandler("~/imu/gyro", gyroscope))
    ros_manager.RegisterHandler(chros.ChROSMagnetometerHandler("~/imu/mag", magnetometer))
    
    # Initialize ROS communication
    ros_manager.Initialize()
    
    # -----------------------------
    # Configure ground body motion
    # -----------------------------
    def update_ground_motion(time):
        """Define motion pattern for the ground body"""
        # Circular motion with sinusoidal vertical component
        radius = 5.0
        angular_freq = 0.5
        
        x = radius * math.cos(angular_freq * time)
        y = radius * math.sin(angular_freq * time)
        z = 1.0 + 0.5 * math.sin(2 * angular_freq * time)
        
        # Set position
        ground.SetPos(chrono.ChVectorD(x, y, z))
        
        # Set velocity for smooth motion
        vx = -radius * angular_freq * math.sin(angular_freq * time)
        vy = radius * angular_freq * math.cos(angular_freq * time)
        vz = angular_freq * math.cos(2 * angular_freq * time)
        ground.SetPos_dt(chrono.ChVectorD(vx, vy, vz))
        
        # Add some rotation
        angle = angular_freq * time
        ground.SetRot(chrono.Q_from_AngZ(angle))
        ground.SetWvel_par(chrono.ChVectorD(0, 0, angular_freq))
    
    # -----------------------------
    # Simulation parameters
    # -----------------------------
    step_size = 0.01  # 10ms timestep
    simulation_time = 0
    max_simulation_time = 30.0  # Run for 30 seconds
    
    # Real-time execution parameters
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
    
    # -----------------------------
    # Simulation loop
    # -----------------------------
    while simulation_time < max_simulation_time:
        # Update ground body motion
        update_ground_motion(simulation_time)
        
        # Update sensors
        manager.Update()
        
        # Advance the simulation
        system.DoStepDynamics(step_size)
        
        # Update ROS communication
        if not ros_manager.Update(simulation_time, step_size):
            break
        
        # Maintain real-time execution
        real_time_timer.Spin(step_size)
        
        # Update simulation time
        simulation_time += step_size
        
        # Print progress every 5 seconds
        if int(simulation_time) % 5 == 0 and simulation_time % 5 < step_size:
            print(f"Simulation time: {simulation_time:.1f}s / {max_simulation_time}s")
            pos = ground.GetPos()
            print(f"Ground position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    
    print("Simulation completed successfully!")
    print("Sensor data has been published to ROS topics throughout the simulation.")

if __name__ == "__main__":
    main()