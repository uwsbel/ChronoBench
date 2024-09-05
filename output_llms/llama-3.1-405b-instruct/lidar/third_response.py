import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math

def main():
    # Create the vehicle system
    vehicle = veh.ChVehicle()
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
    vehicle.SetInitRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    vehicle.SetBrakeType(veh.ChVehicle.BrakeType_SHAFTS)

    # Initialize the vehicle parameters
    vehicle.SetMass(1500)
    vehicle.SetWheelbase(2.7)
    vehicle.SetTrack(1.8)

    # Initialize the powertrain
    powertrain = veh.ChShaftsPowertrain()
    vehicle.SetPowertrain(powertrain)

    # Initialize the suspension
    suspension = veh.ChDoubleWishbone()
    vehicle.SetSuspension(suspension)

    # Initialize the steering
    steering = veh.ChAckermann()
    vehicle.SetSteering(steering)

    # Initialize the brakes
    brakes = veh.ChShaftsBrakes()
    vehicle.SetBrakes(brakes)

    # Initialize the driver
    driver = veh.ChDriver()
    driver.Initialize()

    # Create the terrain
    terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, True)
    terrain.SetPos(chrono.ChVectorD(0, 0, -0.1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))

    # Create the sensor manager
    manager = sens.ChSensorManager(vehicle.GetSystem())

    # Create the lidar sensor
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        vertical_samples,  # Number of vertical channels
        horizontal_fov,  # Horizontal field of view
        max_vert_angle,  # Maximum vertical field of view
        min_vert_angle,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # Create the 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        1,  # only 1 vertical channel for 2D lidar
        horizontal_fov,  # Horizontal field of view
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Create the third person view camera sensor
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        640,  # Width
        480,  # Height
        1.0,  # FOV
        0.01,  # Lag
        1.0  # Collection window
    )
    camera.SetName("Third Person Camera")
    camera.SetOffsetPose(chrono.ChFramed(
        chrono.ChVectorD(-5, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    ))

    # Add the camera to the sensor manager
    manager.AddSensor(camera)

    # Simulate the system
    end_time = 40.0
    step_size = 1e-3
    ch_time = 0.0

    while ch_time < end_time:
        # Synchronize the vehicle, driver, and terrain modules
        vehicle.Synchronize(ch_time)
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)

        # Update the sensor manager
        manager.Update()

        # Advance the vehicle, driver, and terrain modules
        vehicle.Advance(step_size)
        driver.Advance(step_size)
        terrain.Advance(step_size)

        # Perform step of dynamics
        vehicle.GetSystem().DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = vehicle.GetSystem().GetChTime()

# Lidar parameters
noise_model = "NONE"  # No noise model
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1. / update_rate  # typically 1/update rate
sample_radius = 2
divergence_angle = 0.003

main()