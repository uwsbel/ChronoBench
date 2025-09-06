import chrono
import numpy as np
import pychrono
import time
import math
import random

# 1. Initialize PyChrono Environment
try:
    chrono = chrono.ChSystem()
    print("PyChrono environment initialized successfully.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()


# 2. Define Simulation Parameters
# Vehicle Parameters
vehicle_length = 10.0
vehicle_mass = 1500.0
vehicle_speed = 10.0
vehicle_acceleration = 0.1
vehicle_turning_radius = 2.0
vehicle_center_of_mass = 1200.0
vehicle_friction = 0.9

# Terrain Parameters
terrain_width = 5.0
terrain_height = 3.0
terrain_depth = 0.5
terrain_density = 0.01
terrain_color = chrono.ChColor(0.2, 0.8, 0.2) # Dark green

# Sensor Parameters
imu_sensitivity = 0.1
gps_sensitivity = 0.05
imu_resolution = 100
gps_resolution = 10
sensor_data_interval = 0.01 # Seconds

# 3.  Create the Vehicle System
try:
    vehicle = chrono.ChBodyAuxRef()
    vehicle.SetPos(chrono.ChVector3d(0, 0, 0))
    vehicle.SetMass(vehicle_mass)
    vehicle.SetInertiaXX(chrono.ChVector3d(0, 0, 0)) # Inertia is not needed for a simple vehicle
    vehicle.SetFixed(True)
    vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle.SetContactMethod(chrono.ChContactMethod_NonSmooth)
    vehicle.SetChCollisionModel.SetDefaultSuggestedEnvelope(chrono.ChVector3d(0.01, 0.01, 0.01))
    vehicle.SetChCollisionModel.SetDefaultSuggestedMargin(chrono.ChVector3d(0.005, 0.005, 0.005))

    # 4.  Create the IMU and GPS Sensors
    imu = chrono.ChLinkLockLock()
    imu.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT, terrain_width, terrain_height))
    imu.SetNodes(vehicle.GetNodes())
    imu.SetSensorType(chrono.ChSensorType_IMU)
    imu.SetSensorType(chrono.ChSensorType_GPS)
    imu.SetResolution(imu_resolution)
    imu.SetData(chrono.ChData_Float)  # Data is for IMU readings

    gps = chrono.ChLinkLockLock()
    gps.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT, terrain_width, terrain_height))
    gps.SetNodes(vehicle.GetNodes())
    gps.SetSensorType(chrono.ChSensorType_GPS)
    gps.SetResolution(gps_resolution)
    gps.SetData(chrono.ChData_Float)  # Data is for GPS readings

    # 5.  Create the Terrain
    terrain = chrono.ChBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, 0))
    terrain.SetDepth(terrain_depth)
    terrain.SetDensity(terrain_density)
    terrain.SetColor(terrain_color)

    # 6.  Create the Driver Input
    driver = chrono.ChBody()
    driver.SetPos(chrono.ChVector3d(0, 0, 0))
    driver.SetMass(vehicle_mass)
    driver.SetInertiaXX(chrono.ChVector3d(0, 0, 0))
    driver.SetFixed(True)
    driver.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    driver.SetContactMethod(chrono.ChContactMethod_NonSmooth)
    driver.SetChCollisionModel.SetDefaultSuggestedEnvelope(chrono.ChVector3d(0.01, 0.01, 0.01))
    driver.SetChCollisionModel.SetDefaultSuggestedMargin(chrono.ChVector3d(0.005, 0.005, 0.005))

    # 7.  Create the Visualization System
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(chrono.ChSystemNSC())
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Placeholder logo

    # 8.  Initialize the Visualization
    vis.Initialize()

    # 9.  Run the Simulation Loop
    while vis.Run():
        # Simulate Vehicle Movement
        vehicle.SetPos(chrono.ChVector3d(0, 0, 0))
        vehicle.SetSpeed(vehicle_speed)
        vehicle.SetAcceleration(vehicle_acceleration)
        vehicle.SetTurningRadius(vehicle_turning_radius)
        vehicle.SetRotation(chrono.ChVector3d(0, 0, 0))
        time.sleep(0.01)  # Small delay to visualize movement

        # Simulate Sensor Updates
        imu_data = chrono.ChData_Float(imu_sensitivity * time.time())
        gps_data = chrono.ChData_Float(gps_sensitivity * time.time())

        # Update Sensor Data
        imu.SetData(imu_data)
        gps.SetData(gps_data)

        # Update Terrain
        terrain.SetPos(chrono.ChVector3d(0, 0, 0))
        terrain.SetDepth(terrain_depth)
        terrain.SetDensity(terrain_density)
        terrain.SetColor(terrain_color)

        # Print Simulation Information
        print(f"Time: {time.time()}, Vehicle Speed: {vehicle_speed:.2f}, IMU Data: {imu_data}, GPS Data: {gps_data}")

        #  Check for collision
        if vehicle.GetCollisionSystemType() == chrono.ChCollisionSystem.Type_BULLET:
            print("Collision detected!")
            break

        time.sleep(0.01) # Reduced sleep for visualization

    print("Simulation finished.")

except Exception as e:
    print(f"An error occurred: {e}")
    # Clean up resources if needed
    if 'chrono' in locals():
        chrono.Close()
    print("Exiting.")