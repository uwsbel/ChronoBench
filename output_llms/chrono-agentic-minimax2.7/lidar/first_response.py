"""
Lidar scanning of a fixed triangular mesh body.

A Wavefront .obj mesh is loaded as a fixed body. A lidar sensor is attached
to the body via an offset pose that orbits around the mesh each frame.
Demonstrates: ChBodyEasyMesh loading, ChLidarSensor with full filter chain,
and per-step buffer printing.
"""

import os, math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Lidar orbit parameters
orbit_radius = 3.0     # metres from mesh centre
orbit_speed = 0.5      # rad/s

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Mesh body (fixed, triangular .obj) using ChBodyEasyMesh ===
mesh_file = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")
body = chrono.ChBodyEasyMesh(
    mesh_file,
    1000.0,                        # density (kg/m^3)
    True,                          # compute_mass
    True,                          # visualize
    False,                         # collide (no collision needed — fixed static body)
    None,                          # material
)
body.SetFixed(True)
body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(body)

# === Lidar visual marker (small sphere that orbits with the sensor) ===
lidar_marker = chrono.ChBodyEasySphere(0.1, 1000.0, True, False)
lidar_marker.SetFixed(True)
lidar_marker.SetPos(chrono.ChVector3d(orbit_radius, 0, 0))
sys.AddBody(lidar_marker)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Add scene lights for the sensor renderer
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor — attached to the mesh body ===
orbit_angle = [0.0]

lidar_offset_base = chrono.ChFramed(
    chrono.ChVector3d(orbit_radius, 0, 0),
    chrono.QUNIT,
)

lidar = sens.ChLidarSensor(
    body,
    5.0,                                 # update_rate Hz (physical)
    lidar_offset_base,
    800,                                 # horizontal_samples
    1,                                   # vertical_samples (2D scan plane)
    2 * chrono.CH_PI,                    # horizontal_fov rad
    0.0,                                 # max_vert_angle
    0.0,                                 # min_vert_angle
    100.0,                               # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                   # sample_radius
    0.003,                               # vert_divergence_angle
    0.003,                               # hori_divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain — data access only (visualization handled by Irrlicht)
lidar.PushFilter(sens.ChFilterDIAccess())                      # depth + intensity host access
lidar.PushFilter(sens.ChFilterPCfromDepth())                   # XYZI point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                    # XYZI host access
manager.AddSensor(lidar)

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Mesh Scan")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, -5, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging (review-only) ===

# === Main loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # --- update lidar orbit each frame (horizontal orbit in XZ plane) ---
        orbit_angle[0] += orbit_speed * time_step * render_every
        new_x = orbit_radius * math.cos(orbit_angle[0])
        new_z = orbit_radius * math.sin(orbit_angle[0])
        new_offset = chrono.ChFramed(
            chrono.ChVector3d(new_x, 0, new_z),
            chrono.QuatFromAngleAxis(orbit_angle[0], chrono.ChVector3d(0, 1, 0)),
        )
        lidar.SetOffsetPose(new_offset)
        # Move the visual marker to match
        lidar_marker.SetPos(chrono.ChVector3d(new_x, 0, new_z))

        # --- per-frame capture (review-only) ---
        frame += 1

        # --- inner physics batch ---
        for _ in range(render_every):
            manager.Update()   # update all sensors every physics step

            # Print lidar buffer data each step
            di_buf = lidar.GetMostRecentDIBuffer()
            if di_buf.HasData():
                di_data = di_buf.GetDIData()
                n = di_data.size
                print(f"  t={sys.GetChTime():.3f}  lidar_angle={orbit_angle[0]:.3f}  "
                      f"buffer_size={n}")

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break

finally:
    pass  # guard: keep finally non-empty after strip
