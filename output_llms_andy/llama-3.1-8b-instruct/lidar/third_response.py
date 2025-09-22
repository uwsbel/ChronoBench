import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

# -----------------
# Vehicle parameters
# -----------------

# Mass of the vehicle (kg)
m_vehicle = 1500

# Moment of inertia of the vehicle (kg*m^2)
I_vehicle = 5000

# Wheel base (m)
wheel_base = 2.5

# Length of the vehicle (m)
length = 4.5

# Width of the vehicle (m)
width = 1.8

# Height of the vehicle (m)
height = 1.5

# Wheel radius (m)
wheel_radius = 0.3

# Vehicle speed (m/s)
v_vehicle = 10.0

# Vehicle steering angle (rad)
steering_angle = 0.0

# Vehicle acceleration (m/s^2)
a_vehicle = 0.0

# Vehicle angular velocity (rad/s)
omega_vehicle = 0.0

# Vehicle angular acceleration (rad/s^2)
alpha_vehicle = 0.0

# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

def main():
    # Create the vehicle
    vehicle = chrono.ChVehicle()
    vehicle.SetMass(m_vehicle)
    vehicle.SetInertia(I_vehicle)
    vehicle.SetWheelBase(wheel_base)
    vehicle.SetLength(length)
    vehicle.SetWidth(width)
    vehicle.SetHeight(height)
    vehicle.SetWheelRadius(wheel_radius)
    vehicle.SetSpeed(v_vehicle)
    vehicle.SetSteeringAngle(steering_angle)
    vehicle.SetAcceleration(a_vehicle)
    vehicle.SetAngularVelocity(omega_vehicle)
    vehicle.SetAngularAcceleration(alpha_vehicle)

    # Create the terrain
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurface(chrono.ChMaterialParamTireFriction(0.5)))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.png"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetHeight(0.1)

    # Create the sensor manager
    manager = sens.ChSensorManager(vehicle)

    # Create the lidar sensor
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),  # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Create the 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),  # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
        horizontal_samples,     # Number of horizontal samples
        1,                      # only 1 vertical channel for 2D lidar
        horizontal_fov,         # Horizontal field of view
        0.0,                    # Maximum vertical field of view
        0.0,                    # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Create the third person camera sensor
    camera = sens.ChCameraSensor(
        vehicle.GetChassis(),  # Body camera is attached to
        0.0, 0.0, 0.0,  # Camera position
        0.0, 0.0, 0.0,  # Camera orientation
        1.0, 1.0, 1.0  # Camera focal length
    )
    camera.SetName("Third Person Camera")
    camera.SetResolution(640, 480)

    # Create the driver
    driver = chrono.ChDriver()
    driver.SetVehicle(vehicle)
    driver.SetSteeringAngle(steering_angle)
    driver.SetAcceleration(a_vehicle)
    driver.SetAngularVelocity(omega_vehicle)
    driver.SetAngularAcceleration(alpha_vehicle)

    # Create the vehicle system
    vehicle_system = chrono.ChSystemNSC()
    vehicle_system.Add(vehicle)
    vehicle_system.Add(terrain)
    vehicle_system.Add(lidar)
    vehicle_system.Add(lidar_2d)
    vehicle_system.Add(camera)
    vehicle_system.Add(driver)

    # Create the sensor manager
    sensor_manager = sens.ChSensorManager(vehicle_system)

    # Add the lidar to the sensor manager
    sensor_manager.AddSensor(lidar)
    sensor_manager.AddSensor(lidar_2d)

    # Create the simulation loop
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the vehicle
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        sensor_manager.Update()

        # Perform step of dynamics
        vehicle_system.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = vehicle_system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

main()