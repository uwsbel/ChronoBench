import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # Create and initialize an ARTcar vehicle
    vehicle = veh.ChARTcar()
    vehicle.Initialize(mphysicalSystem)

    # Set vehicle parameters
    vehicle.SetChassisCollisionShape(veh.ChCollisionShapeType_BOX, 2.5, 0.5, 0.5)
    vehicle.SetChassisMass(1500)
    vehicle.SetChassisInertia(chrono.ChVector3d(1000, 1000, 1000))

    # Add vehicle to the physical system
    mphysicalSystem.Add(vehicle.GetChassisBody())

    # Initialize a driver for the vehicle with default settings
    driver = veh.ChVehicleDriver(vehicle)
    driver.Initialize()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ----------------------------------
    # Add a mesh to be sensed by a lidar
    # ----------------------------------
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

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
    divergence_angle = 0.003

    # Create a lidar and add it to the sensor manager
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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

    # Create a filter graph for post-processing the data from the lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # Create 2D lidar sensor
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

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Create a third person view camera sensor
    offset_pose_camera = chrono.ChFramed(
        chrono.ChVector3d(0, -5, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        30,  # Update rate
        offset_pose_camera,
        640,  # Horizontal pixels
        480,  # Vertical pixels
        1.0  # FOV
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(1 / 30)

    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person View"))

    manager.AddSensor(camera)

    # Create terrain
    terrain = veh.ChTerrain(mphysicalSystem)
    terrain.Initialize(chrono.GetChronoDataFile("terrain/terrain.jpg"), 400, 400, 0.2, 0.2)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/beton.jpg"))
    terrain.SetMaterial(chrono.ChMaterialDesc().RegisterMaterial())

    # ---------------------
    # Simulation parameters
    # ---------------------
    step_size = 1e-3
    end_time = 40.0
    vis = True
    save = False
    out_dir = "SENSOR_OUTPUT/"

    # ---------------
    # Simulate system
    # ---------------
    t1 = time.time()
    while mphysicalSystem.GetChTime() < end_time:
        # Advance the driver
        driver.Advance(step_size)

        # Advance the vehicle
        vehicle.Advance(step_size)

        # Synchronize the terrain
        terrain.Synchronize(vehicle.GetChassisBody().GetPos(), step_size)

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()
        print("Sim time:", ch_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()