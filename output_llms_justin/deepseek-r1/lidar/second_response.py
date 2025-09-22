import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

# ---------------------
# Simulation parameters
# ---------------------

# Box dimensions (meters)
side = 2.0

# Noise model
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
collection_time = 1. / update_rate

# Radius of samples to use
sample_radius = 2

# Divergence angle
divergence_angle = 0.003

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

def main():
    # Create the system
    mphysicalSystem = chrono.ChSystemNSC()

    # Create box body
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # 1000 kg/m³ density
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    
    # Add texture to the box
    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.GetVisualShape(0).SetTexture(texture)
    
    mphysicalSystem.Add(box_body)

    # Create sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create and configure 3D lidar
    offset_pose_3d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,              # Attached to box
        update_rate,           # Scanning rate
        offset_pose_3d,        # Offset pose
        horizontal_samples,    # Horizontal samples
        vertical_samples,      # Vertical channels
        horizontal_fov,        # Horizontal FOV
        max_vert_angle,        # Max vertical angle
        min_vert_angle,        # Min vertical angle
        100.0,                 # Max range
        sens.BeamShape_RECTANGULAR,  # Beam shape
        sample_radius,         # Sample radius
        divergence_angle,      # Vertical divergence
        divergence_angle,      # Horizontal divergence
        return_mode            # Return mode
    )
    lidar_3d.SetName("3D Lidar")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # Add filters to 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_3d)

    # Create and configure 2D lidar (single vertical channel)
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Attached to box
        update_rate,           # Scanning rate
        offset_pose_2d,        # Offset pose
        horizontal_samples,    # Horizontal samples
        1,                     # Single vertical channel
        horizontal_fov,        # Horizontal FOV
        0.001,                 # Max vertical angle (small slice)
        0,                     # Min vertical angle
        100.0,                 # Max range
        sens.BeamShape_RECTANGULAR,  # Beam shape
        sample_radius,         # Sample radius
        divergence_angle,      # Vertical divergence
        divergence_angle,      # Horizontal divergence
        return_mode            # Return mode
    )
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters to 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Simulation loop
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Update 3D lidar position (orbiting)
        lidar_3d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access data from 3D lidar
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f'3D Lidar: {xyzi_buffer.Width}x{xyzi_buffer.Height} points | Max intensity: {np.max(xyzi_data["intensity"]):.2f}')

        # Update sensor manager
        manager.Update()
        
        # Simulation step
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulation time: {end_time}s | Wall time: {time.time()-t1:.2f}s")

if __name__ == "__main__":
    main()