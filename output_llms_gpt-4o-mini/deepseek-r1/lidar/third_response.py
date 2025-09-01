import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # ---------------------------
    # Create ARTcar and initialize
    # ---------------------------
    vehicle = veh.ARTcar()
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    vehicle.Initialize(veh.ChVehicleVisualSystemType_NONE)
    mphysicalSystem = vehicle.GetSystem()

    # -----------------
    # Initialize driver
    # -----------------
    driver = vehicle.GetDriver()
    driver.Initialize()

    # -------------
    # Create terrain
    # -------------
    terrain = chrono.ChBodyEasyBox(20, 20, 0.1, 1000, True, True)
    terrain.SetPos(chrono.ChVector3d(0, 0, -0.5))
    terrain.SetFixed(True)
    vis_mat = chrono.ChVisualMaterial()
    vis_mat.SetDiffuseColor(chrono.ChColor(0.8, 0.8, 0.5))
    vis_mat.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    terrain.GetVisualModel().GetShapes()[0].SetMaterial(vis_mat)
    terrain.GetVisualModel().GetShapes()[0].SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    mphysicalSystem.Add(terrain)

    # -----------------------
    # Create sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)

    # --------------------------
    # Create and configure lidars
    # --------------------------
    chassis = vehicle.GetChassisBody()
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    # 3D Lidar
    lidar = sens.ChLidarSensor(
        chassis,
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
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # 2D Lidar
    lidar_2d = sens.ChLidarSensor(
        chassis,
        update_rate,
        offset_pose,
        horizontal_samples,
        1,  # Single vertical channel
        horizontal_fov,
        0.0,  # Max vertical angle
        0.0,  # Min vertical angle
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

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # --------------------------
    # Add third-person view camera
    # --------------------------
    cam_pose = chrono.ChFramed(
        chrono.ChVector3d(2, 0, 1),
        chrono.Q_from_AngAxis(0.2, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        chassis,
        30,  # Update rate
        cam_pose,
        1280,  # Resolution
        720,
        chrono.CH_PI / 3  # FOV
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    manager.AddSensor(camera)

    # ---------------
    # Simulation loop
    # ---------------
    ch_time = 0.0
    orbit_radius = 10  # Not used in modified version
    t1 = time.time()

    while ch_time < end_time:
        # Update sensor manager
        manager.Update()

        # Synchronize vehicle systems
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain)
        terrain.Synchronize(ch_time)

        # Advance simulation
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        mphysicalSystem.DoStepDynamics(step_size)

        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Sensor parameters
# -----------------
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0.0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True

if __name__ == "__main__":
    main()