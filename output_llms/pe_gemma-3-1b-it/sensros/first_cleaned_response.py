import chrono
import numpy as np
import os
import math
import sys
import time


try:
    chrono = chrono.ChSystem()
    print("PyChrono environment initialized.")
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()



simulation_name = "GroundSimulation"
simulation_type = "Vehicle"
simulation_speed = 1.0  
simulation_time = 60.0 
simulation_duration = 60.0
simulation_resolution = 100 



ground_body = chrono.ChBody()
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetMass(100.0)
ground_body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
ground_body.SetRayleighDamping(0.000)
ground_body.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(chrono.ChSensor.Camera, "camera", 1) 
sensor_manager.AddSensor(chrono.ChSensor.Lidar, "lidar", 2) 
sensor_manager.AddSensor(chrono.ChSensor.GPS, "gps", 3) 
sensor_manager.AddSensor(chrono.ChSensor.Accelerometer, "accel", 4) 
sensor_manager.AddSensor(chrono.ChSensor.Gyroscope, "gyro", 5) 
sensor_manager.AddSensor(chrono.ChSensor.Magnetometer, "magnet", 6) 



visualization_system = chrono.ChVisualSystemIrrlicht()
visualization_system.SetWindowSize(1024, 768)
visualization_system.SetWindowTitle(simulation_name)
visualization_system.AttachSystem(ground_body)
print("Visualization system initialized.")


collision_settings = chrono.ChCollisionSettings()
collision_settings.SetDefaultSuggestedEnvelope(0.01)
collision_settings.SetDefaultSuggestedMargin(0.005)








try:
    while True:
        
        time.sleep(simulation_speed)

        
        ground_body.SetPos(chrono.ChVector3d(0, 0, 0)) 
        ground_body.SetMass(100.0)
        ground_body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
        ground_body.SetRayleighDamping(0.000)
        ground_body.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

        
        ground_body.SetSensorValues(sensor_manager.GetSensorValues())

        
        print(f"Ground Body Position: {ground_body.GetPos()}")
        print(f"Ground Body Mass: {ground_body.GetMass()}")
        print(f"Ground Body Inertia: {ground_body.GetInertiaXX()}")
        print(f"Ground Body Rayleigh Damping: {ground_body.GetRayleighDamping()}")
        print(f"Ground Body Magnetic Field: {ground_body.GetMagnetometer()}")


        
        ground_body.SetMotionY(chrono.ChFunctionSine(0.001, 1.5))  
        time.sleep(simulation_duration) 

except Exception as e:
    print(f"Simulation Error: {e}")
    
    exit()