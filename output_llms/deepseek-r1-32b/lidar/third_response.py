import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle
import numpy as np
import time
import math

# Lidar parameters
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
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Simulation parameters
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------
    # Create ARTcar vehicle
    # -----------------------
    vehicle.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Create the vehicle
    artcar = vehicle.Artcar vehicle.ChassisFrame(), mphysicalSystem
    artcar.SetChassisMass(1000)
    artcar.SetChassisInertia(chrono.ChVector<>(100, 100, 100))
    artcar.SetChassisDimensions(2, 1, 0.5)
    artcar.SetSuspensionType(vehicle.ArtcarSuspensionType_DOUBLEWISHBONE)
    artcar.SetTireType(vehicle.ArtcarTireType_TMEASY)
    artcar.SetEngineType(vehicle.ArtcarEngineType_SIMPLE)
    artcar.Initialize()

    # Create driver
    driver = vehicle.ChDriver()
    driver.Initialize(artcar.GetChassis(), mphysicalSystem.Get ChronoSystem())

    # Create terrain
    terrain = vehicle.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurface())
    terrain.SetTexture(chrono.GetChronoDataFile('textures/grass.jpg'))
    terrain.SetColor(chrono.ChColor(0.2, 0.3, 0.1))
    terrain.SetSize(100, 100)
    terrain.SetHeight(0)
    mphysicalSystem.Add(terrain)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    # Create 3D lidar sensor
    lidar = sens.ChLidarSensor(
        artcar.GetChassis(),  # Attach to vehicle chassis
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassis(),  # Attach to vehicle chassis
        update_rate,
        offset_pose,
        horizontal_samples,
        1,  # Single vertical channel for 2D
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

    # Add noise model if specified
    noise_model = "NONE"
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    # Add visualization if enabled
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))

    # Add data access filters
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add sensors to manager
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # -----------------------
    # Create third person camera
    # -----------------------
    camera = sens.ChCameraSensor(
        artcar.GetChassis(),
        chrono.ChFramed(chrono.ChVector3d(5, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))),
        640, 480,
        90,  # Horizontal FOV
        60,  # Vertical FOV
        0.1, 1000  # Near and far clipping planes
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera("Third Person View"))
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update driver inputs
        driver.Synchronize(time, step_size)

        # Update vehicle
        artcar.Synchronize(time, step_size)

        # Update terrain
        terrain.Synchronize(time, step_size)

        # Update sensor manager
        manager.Update()

        # Access lidar data
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()