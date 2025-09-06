import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

# -----------------
# Lidar parameters
# -----------------
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

def main():
    # -----------------
    # Create the system
    # -----------------
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChSystemDynamics.ContactMethod.NSC)
    car.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    car.Initialize()

    # Create terrain
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)), 100, 100)
    terrain.Initialize()

    # Initialize driver
    driver = veh.RoboticDriver()
    car.SetDriver(driver)
    driver.Initialize(car.GetVehicle(), step_size)

    mphysicalSystem = car.GetSystem()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # Define lidar offset pose
    lidar_offset = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),  # Changed from -12 to 1.0
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    # -----------------------
    # Create 3D Lidar Sensor
    # -----------------------
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),          # Attached to vehicle chassis
        update_rate,                   # Scanning rate
        lidar_offset,                  # Offset pose
        horizontal_samples,            # Horizontal samples
        vertical_samples,              # Vertical samples
        horizontal_fov,                # Horizontal FOV
        max_vert_angle,                # Max vertical angle
        min_vert_angle,                # Min vertical angle
        100.0,                        # Max range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius,                 # Sample radius
        divergence_angle,              # Divergence angle
        divergence_angle,              # Divergence angle
        return_mode                    # Return mode
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # -----------------------
    # Create 2D Lidar Sensor
    # -----------------------
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(),          # Attached to vehicle chassis
        update_rate,                   # Scanning rate
        lidar_offset,                  # Offset pose
        horizontal_samples,            # Horizontal samples
        1,                             # Vertical samples (2D)
        horizontal_fov,                # Horizontal FOV
        0.0,                           # Max vertical angle (2D)
        0.0,                           # Min vertical angle (2D)
        100.0,                         # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Raw Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # -----------------------
    # Add Third Person Camera
    # -----------------------
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(3.0, 0, 1.5),  # Position relative to chassis
        chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(1,0,0))  # Look down
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),  # Attached to chassis
        30,                    # Update rate (Hz)
        camera_offset,          # Offset pose
        90,                    # Field of view (degrees)
        640, 480               # Resolution
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    # ---------------
    # Simulation Loop
    # ---------------
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Update driver
        driver.Update(ch_time)

        # Update vehicle and terrain
        car.Synchronize(driver.GetInput())
        car.Update(ch_time)
        terrain.Synchronize(ch_time)
        terrain.Update()

        # Update sensor manager
        manager.Update()

        # Step the simulation
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        # Access sensor data (example for 3D lidar)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            data = xyzi_buffer.GetXYZIData()
            print(f"3D Lidar Data: Max Value = {np.max(data)}")

    print(f"Sim time: {end_time} | Wall time: {time.time()-t1}")

if __name__ == "__main__":
    main()

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

# Simulation Parameters
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

# Lidar Parameters
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

def main():
    # Initialize ARTcar vehicle system
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChSystemDynamics.ContactMethod.NSC)
    car.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    car.Initialize()

    # Create terrain
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
    terrain.Initialize()

    # Initialize driver
    driver = veh.RoboticDriver()
    car.SetDriver(driver)
    driver.Initialize(car.GetVehicle(), step_size)

    mphysicalSystem = car.GetSystem()

    # Create sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Lidar offset pose
    lidar_offset = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    # Create 3D Lidar Sensor
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),
        update_rate,
        lidar_offset,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # Create 2D Lidar Sensor
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(),
        update_rate,
        lidar_offset,
        horizontal_samples,
        1,
        horizontal_fov,
        0.0,
        0.0,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Raw"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Add Third Person Camera
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(3.0, 0, 1.5),
        chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),
        30,
        camera_offset,
        90,
        640,
        480
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    # Simulation Loop
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        driver.Update(ch_time)
        car.Synchronize(driver.GetInput())
        car.Update(ch_time)
        terrain.Synchronize(ch_time)
        terrain.Update()
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        # Access sensor data example
        if lidar.GetMostRecentXYZIBuffer().HasData():
            data = lidar.GetMostRecentXYZIBuffer().GetXYZIData()
            print(f"Max 3D Lidar Value: {np.max(data)}")

    print(f"Simulation Time: {end_time} | Wall Time: {time.time() - t1}")

if __name__ == "__main__":
    main()