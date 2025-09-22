import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

# -----------------
# Box parameters
# -----------------
box_side = 10.0  # Dimensions of the box (meters)

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Add a box to be sensed
    # ----------------------------------
    # Create a ChBodyEasyBox. Arguments: half_X, half_Y, half_Z, density, visualize, collide
    box_body = chrono.ChBodyEasyBox(box_side / 2, box_side / 2, box_side / 2, 
                                    1000,      # Density (kg/m^3)
                                    True,      # Enable visualization asset
                                    True)      # Enable collision shape
    box_body.SetPos(chrono.ChVector3d(0, 0, 0)) # Centered at origin
    box_body.SetFixed(True)                   # Make the box stationary
    mphysicalSystem.Add(box_body)

    # Set visual properties for the box
    # ChBodyEasyBox automatically adds a ChVisualShapeBox.
    # We iterate through assets to find the visual shape and set its color.
    vis_asset_found = False
    for asset in box_body.GetAssets():
        if isinstance(asset, chrono.ChVisualShape):
            # Set color to the visual shape
            asset.SetColor(chrono.ChColor(0.5, 0.5, 0.6)) # A greyish-blue color
            
            # Example for setting texture (uncomment and ensure texture file exists):
            # try:
            #     tex = chrono.ChTexture()
            #     tex.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
            #     asset.AddTexture(tex) # Use AddTexture for ChVisualShape
            # except Exception as e:
            #     print(f"Warning: Could not set texture for box - {e}")
            vis_asset_found = True
            break
    if not vis_asset_found:
        print("Warning: Could not find visual asset to set color/texture for the box.")


    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    # Optional: Set manager verbosity
    # manager.SetVerbose(True)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    # This lidar will orbit around the box
    initial_offset_pose_3d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), # Initial position, will be overridden by orbit
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)) # Initial orientation
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,               # Body lidar is attached to (the new box_body)
        update_rate,            # Scanning rate in Hz
        initial_offset_pose_3d, # Initial offset pose (will be updated in the loop)
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius for beam (X-direction)
        sample_radius,          # Sample radius for beam (Y-direction)
        divergence_angle,       # Beam divergence angle (horizontal)
        divergence_angle,       # Beam divergence angle (vertical)
        return_mode             # Return mode for the lidar
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the 3D lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        # Note: Check if ChFilterLidarNoiseXYZI is fully supported in your PyChrono version
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass # No noise filter

    if vis:
        # Visualize the raw lidar data (depth map)
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Raw Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar_3d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D L
print("error happened with only start ```python")