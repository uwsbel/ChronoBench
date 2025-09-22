import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Add a box object instead of a mesh
    # ----------------------------------
    side = 1.0  # size of the box sides
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # mass=1000
    box_body.SetPos(chrono.ChVector3d(0, 0, side / 2))
    # Optional: set texture or color
    # box_body.GetVisualShape(0).SetTextureFilename("some_texture.png")
    mphysicalSystem.Add(box_body)

    # ----------------------------------------------------
    # Create a sensor manager
    # ----------------------------------------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar attached to the box
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    # Define parameters for the 3D lidar
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
    max_vert_angle = chrono.CH_C_PI / 12
    min_vert_angle = -chrono.CH_C_PI / 6
    max_range = 100.0
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    vis = True
    collection_time = 1.0 / update_rate
    lag = 0

    lidar = sens.ChLidarSensor(
        box_body,               # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Max vertical angle
        min_vert_angle,         # Min vertical angle
        max_range,              # Max range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle
        return_mode             # Return mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Create a 2D lidar sensor (single vertical channel)
    # Position it differently if desired
    lidar_2d = sens.ChLidarSensor(
        box_body,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(0, 0, 1.5),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
        horizontal_samples,
        1,  # single vertical channel
        horizontal_fov,
        0,  # vertical angle (0 means 2D scan)
        0,
        max_range,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # ------------------------------------------------
    # Add filters and visualization to the 3D lidar
    # ------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Host access to Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # Host access to XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # Add the 2D lidar sensor with visualization and filters
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    # -----------------
    # Simulation loop
    # -----------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the box
        angle = ch_time * orbit_rate
        new_pos = chrono.ChVector3d(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1
        )
        new_rot = chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1))
        lidar.SetOffsetPose(chrono.ChFramed(new_pos, new_rot))

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager
        manager.Update()

        # Step the simulation
        mphysicalSystem.DoStepDynamics(step_size)

        # Update time
        ch_time = mphysicalSystem.GetChTime()

    print("Simulation completed. Total sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
# Note: Noise models are not implemented in Python version
# Uncomment and implement if available
# noise_model = "CONST_NORMAL_XYZI"
noise_model = "NONE"

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate

# Radius of samples to use
sample_radius = 2

# Divergence angle (sample radius)
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0

save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

if __name__ == "__main__":
    main()