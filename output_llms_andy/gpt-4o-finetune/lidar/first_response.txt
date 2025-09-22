import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens
from matplotlib import pyplot as plt


def main():
    #  Create the physical simulation system: non-smooth contact (NSC)
    mphysicalSystem = chrono.ChSystemNSC()

    # Add a truss to the system. This body will be immobile.
    body_truss = chrono.ChBody()
    body_truss.setFixed(True)
    mphysicalSystem.add(body_truss)

    # Load a triangular mesh (.obj file)
    # The ChTriangleMeshConnected class is used to manage a triangular mesh.
    # By using the .LoadWavefrontMesh() method, you can load a mesh from a Wavefront .obj file.
    # Here, the mesh is scaled and resized to specific dimensions.
    trimesh = chrono.ChTriangleMeshConnected()
    trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    trimesh.Transform(chrono.ChVector3d(-1.5, 0, 0), chrono.ChMatrix33d( 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5))

    # Create a visualization shape using the mesh
    # The ChVisualShapeTriangleMesh class helps visualize a triangular mesh.
    # This visualization can be used to represent the external appearance of a body in the simulation.
    trimesh_shape = chrono.ChVisualShapeTriangleMesh(trimesh)
    trimesh_shape.SetName("HMMWV CHASSIS MESH")
    trimesh_shape.SetMutable(False)  # Indicates the shape is not mutable
    body_truss.AddVisualShape(trimesh_shape)  # Add the shape to the truss body for visualization

    # Create a sensor manager to handle all the sensors in the simulation
    # This manager coordinates the updating and rendering of all added sensors.
    manager = sens.ChSensorManager(mphysicalSystem)

    # Define the update rate for the sensor manager (in Hz)
    update_rate = 15.0
    manager.SetUpdateRate(update_rate)

    # ========================================================================
    # Create a lidar and attach it to the body_truss
    # A lidar (Light Detection and Ranging) sensor is being simulated here.
    # The position and orientation of the lidar relative to the truss body are defined next.
    # ========================================================================
    offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    # Create the lidar sensor and configure its properties
    lidar = sens.ChLidarSensor(body_truss,              # Body lidar is attached to
                               update_rate,              # Scanning rate in Hz
                               offset_pose,              # Pose of the lidar relative to the body
                               100,                      # Number of horizontal samples
                               3.1415926,                 # Horizontal field of view (radians)
                               50,                       # Number of vertical channels
                               chrono.CH_PI / 12,        # Maximum vertical field of view (radians)
                               chrono.CH_PI / 24,        # Minimum vertical field of view (radians)
                               50.0,                     # Maximum sensing distance
                               sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
                               1.0,                      # Lag time
                               2.0 / 1000.0               # Sample time
                               )
    lidar.SetName("Lidar Sensor")
    # Optionally, the return mode of the lidar can be set (commented out here)
    # lidar.SetReturnMode(sens.LidarReturnMode_LAST_RETURN)

    # ========================================================================
    # Create a list of filters for post-processing the lidar data
    # Filters can modify or analyze the data captured by the sensor.
    # ========================================================================
    # Filter out points that do not meet a minimum return threshold.
    lidar.PushFilter(sens.ChFilterLidarReturn())
    # Apply noise model to the lidar data for realism.
    lidar.PushFilter(sens.ChFilterDDA466Noise())
    # Convert raw lidar data to a point cloud representation.
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    # Visualize the point cloud data. This filter typically does not modify the data but provides visualization.
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Point Cloud"))

    # Save the rendered image to a specified directory.
    lidar.PushFilter(sens.ChFilterSave("../SENSOR_OUTPUT/SCENE_LIDAR/"))

    # Access the point cloud data buffer from the lidar
    lidar.PushFilter(sens.ChFilterOnFile("../SENSOR_OUTPUT/PCD_DATA/lidar_data.pcd", "%04.0f"))  # Save data to a file
    # Provides host access to the lidar data, typically required for CPU-based processing or analysis.
    access_buffer = sens.ChFilterAccess()
    lidar.PushFilter(access_buffer)

    # Attach the lidar to the sensor manager
    manager.AttachSensor(lidar)

    # ---------------
    # Simulation loop
    # ---------------
    orbit_radius = 3
    orbit_rate = 0.5
    time = 0
    time_orbit = 2 * 3.1415926 / orbit_rate
    while True:
        time += step_size
        # Dynamically update the lidar's position to orbit around a point in the scene
        new_lidar_pose = chrono.ChFramed(
            chrono.ChVector3d(orbit_radius * math.cos(orbit_rate * time), orbit_radius * math.sin(orbit_rate * time), 1),
            chrono.QuatLookAt(
                (chrono.ChVector3d(orbit_radius * math.cos(orbit_rate * time), orbit_radius * math.sin(orbit_rate * time), 1)),
                (chrono.ChVector3d(0, 0, 1))))
        lidar.SetOffsetPose(new_lidar_pose)

        # Access and print the lidar data
        buffer = access_buffer.GetPCfreeBuffer()
        print(f"Lidar buffer points: {buffer.GetPoints().shape[0]}")

        #  Update the sensor manager (this will render/save/filter the data automatically)
        manager.Update()

        # Perform collision detection and advance the simulation
        mphysicalSystem.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()