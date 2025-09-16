import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # -----------------------
    # Create the vehicle
    # -----------------------
    vehicle = veh.ARTcar()
    vehicle.SetChassisFixed(False)
    vehicle.Initialize(
        system,
        chrono.ChVectorD(0, 0, 0.2),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)),
        veh.InitVehicleDriveTrainType.DEFAULT
    )

    driver = veh.ChDriver()
    vehicle.SetDriver(driver)
    driver.SetThrottle(0.5)
    driver.SetSteering(0)

    # -----------------------
    # Create terrain
    # -----------------------
    terrain = veh.RigidTerrain(
        system,
        veh.ChMaterialSurfaceNSC(),
        True,  # use tangential collisions
        False,  # use color
        None,  # texture
        chrono.ChColor(0.8, 0.8, 0.8)  # color
    )
    terrain.Initialize()

    # -----------------------
    # Create sensor manager
    # -----------------------
    manager = sens.ChSensorManager(system)

    # -----------------------
    # Compute vertical FOV
    # -----------------------
    vertical_fov = max_vert_angle - min_vert_angle

    # -----------------------
    # Create lidar sensors
    # -----------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    # 3D Lidar
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),  # attached to chassis
        update_rate,
        offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        vertical_fov,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    setup_filters(lidar)

    # 2D Lidar
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),
        update_rate,
        offset_pose,
        horizontal_samples,
        1,
        horizontal_fov,
        0.0,  # vertical_fov for 2D
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    setup_filters(lidar_2d, is_2d=True)

    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # -----------------------
    # Add third-person camera
    # -----------------------
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(5, 0, 2),
        chrono.Q_from_AngAxis(math.pi, chrono.ChVectorD(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassis(),
        camera_offset,
        0.1,
        640,
        480,
        60,
        0.1,
        100.0
    )
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Synchronize driver inputs
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time)
        terrain.Synchronize(ch_time)

        # Update vehicle and driver
        vehicle.Update(ch_time)
        driver.SetSteering(0)  # Example control input

        # Update sensor manager
        manager.Update()

        # Advance simulation
        system.DoStepDynamics(step_size)
        ch_time = system.GetChTime()

        # Access lidar data (example)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData() and vis:
            print('Received lidar data:', xyzi_buffer.Width, xyzi_buffer.Height)

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

def setup_filters(sensor, is_2d=False):
    if noise_model == "CONST_NORMAL_XYZI":
        sensor.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    sensor.PushFilter(sens.ChFilterDIAccess())
    sensor.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        if is_2d:
            sensor.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
        else:
            sensor.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    sensor.PushFilter(sens.ChFilterXYZIAccess())

# -----------------
# Lidar parameters
# -----------------
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
vertical_fov = max_vert_angle - min_vert_angle  # Precompute
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

main()

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # Create the system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create the vehicle
    vehicle = veh.ARTcar()
    vehicle.SetChassisFixed(False)
    vehicle.Initialize(
        system,
        chrono.ChVectorD(0, 0, 0.2),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0)),
        veh.InitVehicleDriveTrainType.DEFAULT
    )

    driver = veh.ChDriver()
    vehicle.SetDriver(driver)
    driver.SetThrottle(0.5)
    driver.SetSteering(0)

    # Create terrain
    terrain = veh.RigidTerrain(
        system,
        veh.ChMaterialSurfaceNSC(),
        True,
        False,
        None,
        chrono.ChColor(0.8, 0.8, 0.8)
    )
    terrain.Initialize()

    # Create sensor manager
    manager = sens.ChSensorManager(system)

    # Compute vertical FOV
    vertical_fov = max_vert_angle - min_vert_angle

    # Create lidar sensors
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    # 3D Lidar
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),
        update_rate,
        offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        vertical_fov,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    setup_filters(lidar)

    # 2D Lidar
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),
        update_rate,
        offset_pose,
        horizontal_samples,
        1,
        horizontal_fov,
        0.0,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    setup_filters(lidar_2d, is_2d=True)

    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # Add third-person camera
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(5, 0, 2),
        chrono.Q_from_AngAxis(math.pi, chrono.ChVectorD(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassis(),
        camera_offset,
        0.1,
        640,
        480,
        60,
        0.1,
        100.0
    )
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    # Simulation loop
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time)
        terrain.Synchronize(ch_time)

        vehicle.Update(ch_time)
        driver.SetSteering(0)

        manager.Update()

        system.DoStepDynamics(step_size)
        ch_time = system.GetChTime()

        # Access lidar data (example)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData() and vis:
            print('Received lidar data:', xyzi_buffer.Width, xyzi_buffer.Height)

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

def setup_filters(sensor, is_2d=False):
    if noise_model == "CONST_NORMAL_XYZI":
        sensor.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    sensor.PushFilter(sens.ChFilterDIAccess())
    sensor.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        if is_2d:
            sensor.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar"))
        else:
            sensor.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar"))
    sensor.PushFilter(sens.ChFilterXYZIAccess())

# Lidar parameters
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
vertical_fov = max_vert_angle - min_vert_angle
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# Simulation parameters
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

main()