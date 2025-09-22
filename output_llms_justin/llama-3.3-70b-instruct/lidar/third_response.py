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
    vehicle = veh.ChVehicle()

    # Set vehicle parameters
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisFixed(False)

    # Initialize the vehicle
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))

    # Add the vehicle to the physical system
    mphysicalSystem.Add(vehicle.GetChassisBody())

    # Initialize a driver for the vehicle
    driver = veh.ChIrrlichtDriver()
    driver.SetVehicle(vehicle)
    driver.Initialize()

    # Create a rigid terrain
    terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000, True)
    terrain.SetPos(chrono.ChVectorD(0, 0, 0))
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    mphysicalSystem.Add(terrain)

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create a 3D lidar sensor
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngX(0)
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,     # Number of horizontal samples
        300,       # Number of vertical channels
        2 * chrono.CH_PI,         # Horizontal field of view
        chrono.CH_PI / 12,         # Maximum vertical field of view
        -chrono.CH_PI / 6,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,          # Sample radius
        0.003,       # Divergence angle
        0.003,       # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5.0)

    # Add filters to the lidar sensor
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar sensor to the sensor manager
    manager.AddSensor(lidar)

    # Create a 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,     # Number of horizontal samples
        1,       # Number of vertical channels
        2 * chrono.CH_PI,         # Horizontal field of view
        0.0,         # Maximum vertical field of view
        0.0,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,          # Sample radius
        0.003,       # Divergence angle
        0.003,       # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5.0)

    # Add filters to the 2D lidar sensor
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 2D lidar sensor to the sensor manager
    manager.AddSensor(lidar_2d)

    # Create a third person view camera sensor
    camera_pose = chrono.ChFramed(
        chrono.ChVectorD(0, 0, 5), chrono.Q_from_AngX(0)
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        30.0,            # Frame rate in Hz
        camera_pose,            # Offset pose
        640,     # Image width
        480,       # Image height
        60.0         # Field of view
    )
    camera.SetName("Camera Sensor")

    # Add the camera sensor to the sensor manager
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < 40.0:
        # Set lidar to orbit around the mesh body
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.Q_from_AngX(ch_time * orbit_rate)
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(1e-3)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

        # Advance the vehicle, driver, and terrain modules
        vehicle.Synchronize(1e-3)
        driver.Synchronize(1e-3)
        terrain.Synchronize(1e-3)

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)


if __name__ == '__main__':
    main()