import pychrono.core as chrono
import pychrono.sensor as sens

import pychrono.vehicle as veh
import pychrono.vehicle.terrain as veh_terrain
import pychrono.vehicle.driver as veh_driver

import numpy as np
import time
import math
import os


def main():
    # -----------------
    # Create the vehicle system (ARTiculated CAR)
    # -----------------
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Ensure data path is set

    vehicle = veh.ArticulatedVehicle()
    vehicle.SetContactMethod(chrono.ChMaterialSurfaceSMC.ContactMethod.SMC)
    vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(step_size)
    vehicle.Initialize()

    # Use vehicle system as main physical system
    mphysicalSystem = vehicle.GetSystem()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ----------------------------------
    # Create a rigid terrain
    # ----------------------------------
    terrain = veh_terrain.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch_mat.SetPoissonRatio(0.3)
    patch_mat.SetSlipCompliance(1e-5)
    patch_mat.SetRollingFriction(0.02)
    patch_mat.SetSpinningFriction(0.02)

    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChVectorD(0, 0, 0), 
                             chrono.ChVectorD(0, 0, 1), 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), 200, 200)
    terrain.Initialize()

    # ------------------------------------------
    # Initialize the driver with default settings
    # ------------------------------------------
    driver = veh.ChWillemsDriver(vehicle)
    driver.Initialize()

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # Attach lidar sensors to vehicle chassis instead of box
    # ------------------------------------------------

    # Use the vehicle chassis body to attach sensors
    chassis = vehicle.GetChassisBody()

    # Changed lidar offset pose as instructed
    offset_pose_3d = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar = sens.ChLidarSensor(
        chassis,                  # Body lidar is attached to
        update_rate,              # Scanning rate in Hz
        offset_pose_3d,           # Offset pose
        horizontal_samples,       # Number of horizontal samples
        vertical_samples,         # Number of vertical channels
        horizontal_fov,           # Horizontal field of view
        max_vert_angle,           # Maximum vertical field of view
        min_vert_angle,           # Minimum vertical field of view
        100.0,                    # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,            # Sample radius
        divergence_angle,         # Divergence angle
        divergence_angle,         # Divergence angle (again, typically same value)
        return_mode               # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # ----------------------
    # Create 2D Lidar Sensor
    # ----------------------
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = sens.ChLidarSensor(
        chassis,                # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose_2d,         # Offset pose
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
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # -------------------------------------
    # Add third person camera to the vehicle (chassis)
    # Attach a camera sensor to the chassis with a third person offset pose
    # -------------------------------------
    third_person_offset = chrono.ChFrameD(
        chrono.ChVectorD(-6, 0, 3),  # behind and above the vehicle
        chrono.ChQuaternionD(chrono.ChVectorD(0, 0, 1), math.radians(180))  # Looking at vehicle front
    )
    camera = sens.ChCameraSensor(
        chassis,
        30,                         # update rate in Hz
        third_person_offset,
        640,                        # image width
        480,                        # image height
        math.radians(60)            # vertical FOV in radians (~60 degrees)
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person Camera"))
    if save:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        camera.PushFilter(sens.ChFilterSave(out_dir))
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10  # For lidar orbit -- unused here since lidar is attached to vehicle chassis and we move vehicle
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        # ---------------------------------------------------------------------
        # For completeness, keep the lidar on the chassis at fixed offset pose.
        # As per instruction, lidar offset pose is fixed to vehicle chassis.
        # Could update pose here to orbit if desired; skipping since instruction says attach to chassis.
        # ---------------------------------------------------------------------

        # Update modules (driver, terrain, vehicle, sensor manager)

        time_step = step_size

        driver_inputs = driver.GetInputs()
        driver.Synchronize(ch_time)
        driver.Advance(time_step)

        terrain.Synchronize(ch_time)
        terrain.Advance(time_step)

        vehicle.Synchronize(ch_time, driver_inputs, terrain)
        vehicle.Advance(time_step)

        manager.Update()

        # Step dynamics of system directly:
        mphysicalSystem.DoStepDynamics(time_step)

        ch_time = mphysicalSystem.GetChTime()

        # Access and print lidar data for 3D lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                f"Time {ch_time:.3f} s - 3D Lidar buffer received. Resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}"
            )
            print(f"Max Value: {np.max(xyzi_data):.3f}")

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.LidarReturnMode_MEAN_RETURN
# return_mode = sens.LidarReturnMode_FIRST_RETURN
# return_mode = sens.LidarReturnMode_LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees, replaced CH_PI with CH_C_PI per chrono-python naming
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1.0 / update_rate  # typically 1/update rate

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

if __name__ == "__main__":
    main()