import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # Create the vehicle
    vehicle = veh.ChVehicle(mphysicalSystem)

    # Set vehicle parameters
    vehicle.SetVehicleFile("path/to/vehicle/file")
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisBody(chrono.ChBodyEasyBox(1, 1, 1, 1000))
    vehicle.SetChassisCollide(True)
    vehicle.SetSuspensionMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetTireMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetSteeringMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetBrakeMaterial(chrono.ChMaterialSurfaceNSC())

    # Initialize the vehicle
    vehicle.Initialize()

    # Create the driver
    driver = veh.ChIrrVehicleDriver(vehicle)

    # Create the terrain
    terrain = chrono.ChBodyEasyBox(100, 1, 100, 1000)
    terrain.SetPos(chrono.ChVector3d(0, -1, 0))
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.png"))
    terrain.SetColor(chrono.ChVectorD(0.5, 0.5, 0.5))
    mphysicalSystem.Add(terrain)

    # Create the sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create the lidar sensors
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
    lidar.SetCollectionWindow(1. / 5.0)

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
    lidar_2d.SetCollectionWindow(1. / 5.0)

    # Add filters to the lidar sensors
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar sensors to the sensor manager
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # Create a third person camera
    camera_pose = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        30.0,  # Frame rate in Hz
        camera_pose,  # Offset pose
        640,  # Image width
        480,  # Image height
        60.0,  # Field of view
        sens.CameraSensorMode_COLOR  # Camera mode
    )
    camera.SetName("Third Person Camera")

    # Add the camera to the sensor manager
    manager.AddSensor(camera)

    # Simulate the system
    ch_time = 0.0
    step_size = 1e-3
    end_time = 40.0

    while ch_time < end_time:
        # Update the vehicle
        vehicle.Synchronize(1e-3)
        vehicle.Advance(1e-3)

        # Update the driver
        driver.Synchronize(1e-3)
        driver.Advance(1e-3)

        # Update the terrain
        terrain.Synchronize(1e-3)
        terrain.Advance(1e-3)

        # Update the sensor manager
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time)


if __name__ == "__main__":
    main()