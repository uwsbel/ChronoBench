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
    # Add a mesh to be sensed by a lidar
    # ----------------------------------
    mmesh = chrono.ChBoxMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    # Scale the mesh to a different size
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    # Create a visual representation of the mesh
    trimesh_shape = chrono.ChVisualShapeBoxMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Box")
    trimesh_shape.SetMutable(False)

    # Create a body to hold the mesh
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ----------------------------------
    # Create a lidar and add it to the sensor manager
    # ----------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        mesh_body,
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
        return_mode = "STRONGEST_RETURN"
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # ------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -------------------------------------------------
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

    if vis:
        # Visualize the depth, intensity data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Depth and Intensity Data"))

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

    # The path to the Chrono data directory containing various assets (meshes, textures, data files)
    # is automatically set, relative to the default location of this demo.
    # If running from a different directory, you must change the path to the data directory with:
    # chrono.SetChronoDataPath('path/to/data')

if __name__ == "__main__":
    main()