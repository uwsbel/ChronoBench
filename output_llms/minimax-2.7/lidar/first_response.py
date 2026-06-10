#!/usr/bin/env python3
"""
PyChrono – Lidar scan of a fixed triangular mesh while the sensor orbits.

Features
--------
* Fixed body with a visual triangular mesh loaded from a Wavefront .obj file.
* Lidar sensor attached to a moving body that orbits the mesh.
* Gaussian noise, visualisation (point‑cloud) and CSV saving are enabled.
* Simulation loop prints the lidar buffer (number of points + first few points)
  at each step.
"""

import os
import math
import pychrono as chrono
import pychrono.sensor as sens


def main():
    # ------------------------------------------------------------------
    # 1.  Initialise PyChrono and the data path
    # ------------------------------------------------------------------
    # The data path is where Chrono looks for assets (meshes, shaders, …)
    chrono.SetDataPath(os.path.join(os.path.dirname(__file__), "data/"))

    # ------------------------------------------------------------------
    # 2.  Create a physical system (NSC contact model, bullet collision)
    # ------------------------------------------------------------------
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.Set_G_acc(chrono.Vector(0, 0, -9.81))

    # Optional ground plane (purely visual)
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.Vector(0, 0, 0))
    ground.SetCollide(False)
    system.AddBody(ground)

    # ------------------------------------------------------------------
    # 3.  Fixed body carrying the triangular mesh
    # ------------------------------------------------------------------
    # Path to the .obj file – replace with your own file if needed.
    obj_file = os.path.join(os.path.dirname(__file__), "mesh.obj")
    if not os.path.isfile(obj_file):
        # Try to locate a shipped example mesh (if it exists)
        obj_file = chrono.GetChronoDataFile("sensor/tri.obj")
        if not os.path.isfile(obj_file):
            raise FileNotFoundError(
                "Could not locate an .obj mesh. "
                "Please provide a Wavefront file named 'mesh.obj' in the script directory."
            )

    mesh_body = chrono.ChBody()
    mesh_body.SetFixed(True)                # <‑‑ fixed in the world
    mesh_body.SetPos(chrono.Vector(0, 0, 0))
    mesh_body.SetCollide(False)             # no collision needed – only visual

    # Load the triangle mesh from the OBJ file
    tri_mesh = chrono.ChTriangleMeshConnected()
    tri_mesh.LoadWavefront(obj_file, normalized=True, load_normals=True)

    # Attach a visual shape (rendered by the 3D viewer)
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(tri_mesh)
    vis_shape.SetColor(chrono.ChColor(0.8, 0.5, 0.2))   # orange‑ish
    mesh_body.AddVisualShape(vis_shape)
    system.AddBody(mesh_body)

    # ------------------------------------------------------------------
    # 4.  Moving body that will carry the lidar sensor (orbits the mesh)
    # ------------------------------------------------------------------
    sensor_parent = chrono.ChBody()
    sensor_parent.SetMass(1.0)
    sensor_parent.SetBodyFixed(True)   # we will drive it manually – treat as kinematic
    sensor_parent.SetCollide(False)
    sensor_parent.SetPos(chrono.Vector(5.0, 0.0, 1.5))   # start on the orbit
    system.AddBody(sensor_parent)

    # ------------------------------------------------------------------
    # 5.  Sensor manager
    # ------------------------------------------------------------------
    manager = sens.ChSensorManager(system)

    # ------------------------------------------------------------------
    # 6.  Lidar sensor configuration
    # ------------------------------------------------------------------
    update_rate = 30.0                     # Hz
    max_range = 50.0                       # m
    horizontal_fov = 360.0                 # degrees (full circle)
    vertical_fov = 60.0                    # degrees (looking up & down)
    angular_res = 1.0                       # degrees between rays

    lidar_params = sens.ChLidarSensorParameters()
    lidar_params.SetMaxRange(max_range)
    lidar_params.SetLidarReturnMode(sens.LidarReturnMode_STRONGEST_RETURN)
    lidar_params.SetScanAngle(horizontal_fov)
    lidar_params.SetVerticalScanAngle(vertical_fov)
    lidar_params.SetAngularResolution(angular_res)
    lidar_params.SetPayloadSampleRadius(0.05)   # radius for hit detection
    lidar_params.SetAttenuation(0.025)          # optional attenuation

    # --- Noise model (Gaussian) ----------------------------------------
    noise = sens.ChNoiseGauss()
    noise.SetMean(0.0)
    noise.SetSTD(0.01)
    lidar_params.SetNoiseModel(noise)

    # Offset of the sensor relative to the parent body (here the origin)
    sensor_offset = chrono.ChFrameD(chrono.Vector(0, 0, 0), chrono.Q_from_AngZ(0))

    # Create the lidar sensor and attach it to the moving body
    lidar = sens.ChLidarSensor(sensor_parent, update_rate, sensor_offset, lidar_params)

    # Visualisation of the sensor frustum in the 3‑D view
    lidar.SetRenderActive(True)
    lidar.SetDrawSensorMesh(True)

    # ------------------------------------------------------------------
    # 7.  Visualisation & data‑saving options
    # ------------------------------------------------------------------
    # Show the generated point cloud
    manager.SetLidarRenderMode(sens.LidarRenderMode_PointCloud)

    # Save point‑cloud data to CSV files
    lidar.SetDataSavingActive(True)
    lidar.SetDataSaveFolder("./lidar_output")
    lidar.SetDataSaveBaseFileName("lidar")
    lidar.SetDataSaveFormat(sens.DataFileFormat_CSV)

    # Add the sensor to the manager
    manager.AddSensor(lidar)

    # ------------------------------------------------------------------
    # 8.  Simulation loop – orbit the sensor and print the buffer
    # ------------------------------------------------------------------
    step_size = 1e-3          # 1 ms
    total_time = 5.0          # simulate 5 s
    orbit_radius = 5.0        # metres
    orbit_height = 1.5        # metres (height above the mesh)
    orbit_speed = 1.0         # rad / s

    t = 0.0
    while t < total_time:
        # ---- Update sensor manager (produces data at the chosen update rate) ----
        manager.Update()

        # ---- Kinematically drive the parent body on a circular orbit ---------
        angle = orbit_speed * t
        x = orbit_radius * math.cos(angle)
        y = orbit_radius * math.sin(angle)
        sensor_parent.SetPos(chrono.Vector(x, y, orbit_height))

        # Optional: make the sensor always look at the mesh centre (origin)
        # dir_to_center = -sensor_parent.GetPos()
        # dir_to_center.Normalize()
        # q = chrono.Q_from_AxialRot(chrono.Vector(0, 0, 1), dir_to_center)
        # sensor_parent.SetRot(q)

        # ---- Advance dynamics ------------------------------------------------
        system.DoStepDynamics(step_size)

        # ---- Retrieve most recent lidar buffer --------------------------------
        data = lidar.GetMostRecentDataBuffer()
        if data:
            points = data.Points          # list of chrono.Vector
            print(f"Time {t:.3f} | Points: {len(points)}")
            # Show first 5 points for quick inspection
            for i, pt in enumerate(points[:5]):
                print(f"  [{i}] ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})")
        else:
            print(f"Time {t:.3f} | No data (sensor not updated yet)")

        t += step_size

    print("\nSimulation finished – lidar data saved in ./lidar_output/")

if __name__ == "__main__":
    main()