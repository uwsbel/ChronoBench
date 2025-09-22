import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system with ARTcar vehicle
    # -----------------
    # Initialize the vehicle system
    my_vehicle = veh.ChVehicleAssembly()
    # Create the chassis
    chassis = veh.ChRigidBody()
    chassis.SetPos(chrono.ChVector3d(0, 0, 0.5))
    chassis.SetMass(800)
    chassis.SetInertiaXX(chrono.ChVector3d(1000, 1000, 1000))
    # Add visual shape to chassis
    chassis_shape = chrono.ChBoxShape()
    chassis_shape.GetBoxGeometry().Size = chrono.ChVector3d(1, 0.5, 0.2)
    chassis.GetVisualModel().AddShape(chassis_shape)
    # Add chassis to vehicle
    my_vehicle.Add(chassis)

    # Initialize the vehicle driver
    driver = veh.ChIrrGuiDriver()
    driver.Initialize(chassis)

    # Create terrain
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.8)
    terrain = chrono.ChBodyEasyBox(50, 50, 0.2, 1000, True, True)
    terrain.SetPos(chrono.ChVector3d(0, 0, -0.1))
    terrain.SetMaterialSurface(terrain_material)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"))
    terrain.SetColor(chrono.ChColor(0.3, 0.8, 0.3))
    # Create system and add terrain
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.Add(terrain)

    # Attach vehicle to the system
    my_vehicle.SetSystem(mphysicalSystem)

    # Initialize the vehicle
    my_vehicle.Initialize()

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # -----------------------
    # Create a lidar and add it to the sensor manager
    # -----------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        chassis,              # Body lidar is attached to
        update_rate,        # Scanning rate in Hz
        offset_pose,        # Offset pose
        horizontal_samples, # Number of horizontal samples
        vertical_samples,   # Number of vertical channels
        horizontal_fov,     # Horizontal field of view
        max_vert_angle,     # Max vertical angle
        min_vert_angle,     # Min vertical angle
        100.0,              # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Filter configuration
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
    manager.AddSensor(lidar)

    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        chassis,
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
    elif noise_model == "NONE":
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Add a third person camera sensor attached to chassis
    camera_pose = chrono.ChFrame<double>(
        chrono.ChVector3d(0, -2, 2),
        chrono.QuatFromEulerAngles(0, math.pi / 4, 0)
    )
    camera_sensor = sens.ChCameraSensor(
        chassis,
        update_rate,
        camera_pose,
        640,
        480,
        45  # Horizontal field of view in degrees
    )
    camera_sensor.SetName("Third Person Camera")
    manager.AddSensor(camera_sensor)

    # -----------------
    # Simulation parameters
    # -----------------
    step_size = 1e-3
    end_time = 40.0
    collection_time = 1. / update_rate
    # Simulation loop
    ch_time = 0.0
    t1 = time.time()

    orbit_radius = 10
    orbit_rate = 0.1

    while ch_time < end_time:
        # Update vehicle dynamics
        my_vehicle.Synchronize(ch_time)
        my_vehicle.Advance(step_size)
        driver.Synchronize(ch_time)
        driver.Advance(step_size)
        mphysicalSystem.DoStepDynamics(step_size)

        # Update terrain (if any dynamics)
        # (Terrain is static in this case)

        # Update sensor manager
        # Set lidar to orbit around the vehicle
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    1.0,
                    0,
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Get the most recent XYZI data
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Synchronize driver
        driver.InputSetSpeed(0)  # or set desired speed
        driver.SetSteering(0)    # or set desired steering

        # Update the sensor manager
        manager.Update()

        # Advance the system
        ch_time += step_size

    print("Simulation completed in wall time:", time.time() - t1)

# -----------------
# Lidar parameters
# -----------------
noise_model = "NONE"  # or "CONST_NORMAL_XYZI"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * math.pi  # 360 degrees
max_vert_angle = math.pi / 12
min_vert_angle = -math.pi / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

main()