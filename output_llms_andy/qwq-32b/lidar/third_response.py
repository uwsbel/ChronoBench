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
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------
    # Create the vehicle
    # -----------------
    vehicle = veh.ARTcar_vehicle()
    driver = veh.ChDriver()

    # -----------------
    # Create terrain
    # -----------------
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.9)
    material.SetRestitution(0.01)

    terrain = veh.RigidTerrain(mphysicalSystem)
    patch = terrain.AddPatch(material,
                            chrono.ChVectorD(0, 0, 0),
                            chrono.ChVectorD(1, 0, 0),
                            chrono.ChVectorD(0, 0, 1),
                            1000, 1000,  # length, width
                            True)  # texture coordinates
    patch.texture_path = chrono.GetChronoDataFile("textures/concrete.jpg")
    patch_color = chrono.ChColorAsset()
    patch_color.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    patch.GetCollisionModel().GetAssets().append(patch_color)

    # Initialize vehicle
    vehicle.Initialize(mphysicalSystem,
                      chrono.ChVectorD(0, 0, 0.2),  # Slightly above ground
                      chrono.ChQuaternionD(1, 0, 0, 0),
                      driver,
                      terrain)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # --------------------------------
    # Create lidar offset pose
    # --------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    # -------------------------------
    # Create 3D Lidar Sensor
    # -------------------------------
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Attached to chassis
        update_rate,
        offset_pose,
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
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # -------------------------------
    # Create 2D Lidar Sensor
    # -------------------------------
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),
        update_rate,
        offset_pose,
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
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # -------------------------------
    # Add Third-Person Camera Sensor
    # -------------------------------
    if vis:
        camera_pose = chrono.ChFrameD(
            chrono.ChVectorD(5, 5, 3),
            chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(1, 0, 0))
        )
        camera = sens.ChCameraSensor(
            vehicle.GetChassisBody(),
            30,  # Field of view
            640, 480,  # Resolution
            camera_pose
        )
        camera.SetName("Third Person Camera")
        manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update driver and vehicle
        driver.Update()
        vehicle.Synchronize(driver, terrain)
        vehicle.Update(chrono.GetChTime())

        # Update sensors and system
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        # Access lidar data (optional)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

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
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True

main()