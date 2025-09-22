import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicles as veh
import pychrono.fea as fea

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    # Create an ARTcar vehicle
    my_vehicle = veh.VehicleARTIC()
    my_vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetChassisMass(1500)

    # Initialize the vehicle
    my_vehicle.Initialize(mphysicalSystem)

    # Use the vehicle system as the main physical system
    mphysicalSystem = my_vehicle.GetVehicleSystem()

    # ----------------------------------
    # Add a terrain to the system
    # ----------------------------------
    terrain_mat = fea.ChMaterialSurfaceNSC()
    terrain_mat.SetYoungModulus(5e7)
    terrain_mat.SetFriction(0.5)
    terrain_mat.SetRestitution(0.1)

    terrain_mesh = chrono.ChTriangleMeshConnected()
    terrain_mesh.LoadMesh("terrain.obj")

    terrain = chrono.ChBodyEasyBox(100, 10, 10, 1000, True, True, terrain_mesh, terrain_mat)
    terrain.SetPos(chrono.ChVectorD(0, -5, 0))
    mphysicalSystem.AddBody(terrain)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        my_vehicle.GetChassis(),  # Attach lidar to the vehicle chassis
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

    # Add filters for the 3D lidar
    # ...

    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    lidar_2d = sens.ChLidarSensor(
        my_vehicle.GetChassis(),  # Attach lidar to the vehicle chassis
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

    # Add filters for the 2D lidar
    # ...

    manager.AddSensor(lidar_2d)

    # -----------------------
    # Create a camera sensor
    # -----------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(5, 5, 5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        my_vehicle.GetChassis(),  # Attach camera to the vehicle chassis
        update_rate,
        offset_pose,
        640,
        480,
        60,
        True
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)

    # Add filters for the camera
    # ...

    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    my_driver = veh.ChVehicleDriver(my_vehicle)
    my_driver.SetDefaultCommand(veh.ChVehicleDriverCommand.CMD_ACCELERATE)

    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Synchronize and advance the vehicle, driver, and terrain modules
        my_vehicle.Synchronize(ch_time, step_size)
        my_driver.Synchronize(ch_time, step_size)
        terrain.Synchronize(ch_time, step_size)

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"

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


main()