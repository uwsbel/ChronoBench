import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # Create the system
    mphysicalSystem = chrono.ChSystemNSC()

    # Create the vehicle
    vehicle = veh.ArticulatedVehicle(mphysicalSystem)

    # Initialize the vehicle
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))

    # Set the vehicle parameters
    vehicle.SetVehicleFile("path_to_vehicle_file.json")
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetTireMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetWheelMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetSuspensionMaterial(chrono.ChMaterialSurfaceNSC())

    # Add a driver to the vehicle
    driver = veh.ChIrrlichtDriver()
    vehicle.AddDriver(driver)

    # Create a rigid terrain
    terrain = chrono.ChBodyEasyBox(mphysicalSystem, 100, 100, 1, 1000, True, chrono.ChVectorD(0, -5, 0))
    terrain.GetMaterialSurface().SetFriction(0.9)
    terrain.GetMaterialSurface().SetRestitution(0.8)
    terrain.GetMaterialSurface().SetCompliance(0.0001)
    terrain.GetMaterialSurface().SetDampingF(0.3)
    terrain.GetMaterialSurface().SetDampingV(0.3)

    # Add the terrain to the system
    mphysicalSystem.Add(terrain)

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create a lidar sensor
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngX(0)
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        800,  # Number of horizontal samples
        300,  # Number of vertical channels
        2 * chrono.CH_PI,  # Horizontal field of view
        chrono.CH_PI / 12,  # Maximum vertical field of view
        -chrono.CH_PI / 6,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,  # Sample radius
        0.003,  # Divergence angle
        0.003,  # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5.0)  # typically 1/update rate

    # Add filters to the lidar
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # Create a 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        800,  # Number of horizontal samples
        1,  # only 1 vertical channel for 2D lidar
        2 * chrono.CH_PI,  # Horizontal field of view
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,  # Sample radius
        0.003,  # Divergence angle
        0.003,  # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5.0)  # typically 1/update rate

    # Add filters to the 2D lidar
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Create a third person camera sensor
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        30.0,  # Frame rate in Hz
        chrono.ChFramed(
            chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngX(0)
        ),  # Offset pose
        640,  # Horizontal resolution
        480,  # Vertical resolution
        1.0,  # Field of view
        "Third Person Camera"  # Name of the camera
    )
    camera.SetCollectionWindow(1. / 30.0)  # typically 1/frame rate

    # Add the camera to the sensor manager
    manager.AddSensor(camera)

    # Simulate the system
    step_size = 1e-3
    end_time = 40.0
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Update the vehicle
        vehicle.Synchronize(chrono.ChTime(ch_time))
        vehicle.Advance(step_size)

        # Update the driver
        driver.Synchronize(chrono.ChTime(ch_time))
        driver.Advance(step_size)

        # Update the terrain
        terrain.Synchronize(chrono.ChTime(ch_time))
        terrain.Advance(step_size)

        # Update the sensor manager
        manager.Update()

        # Perform a step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

main()