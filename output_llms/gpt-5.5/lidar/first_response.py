# PyChrono simulation:
# - Loads a Wavefront OBJ triangular mesh as a fixed visual body
# - Adds a lidar sensor using Chrono::Sensor
# - Applies lidar noise, point-cloud visualization, point-cloud saving, and buffer access
# - Dynamically moves the lidar in an orbit around the mesh
# - Prints lidar XYZI buffer data during the simulation

import os
import math
import numpy as np

import pychrono as chrono
import pychrono.sensor as sens


# ---------------------------------------------------------------------
# User parameters
# ---------------------------------------------------------------------

# Replace this with your own Wavefront OBJ file.
# Example:
# OBJ_FILE = "/absolute/path/to/your/mesh.obj"
OBJ_FILE = chrono.GetChronoDataFile("sensor/box.obj")

OUTPUT_DIR = "SENSOR_OUTPUT/lidar_orbit"
os.makedirs(OUTPUT_DIR, exist_ok=True)

simulation_time = 10.0
step_size = 1.0e-3

# Lidar parameters
lidar_update_rate = 10.0          # Hz
horizontal_samples = 1024
vertical_samples = 64
horizontal_fov = 2.0 * math.pi    # 360 deg
vertical_fov_upper = math.radians(10.0)
vertical_fov_lower = math.radians(-30.0)
max_lidar_range = 100.0

# Orbit parameters
orbit_radius = 6.0
orbit_height = 2.0
orbit_angular_speed = 0.5         # rad/s
mesh_center = chrono.ChVector3d(0.0, 0.0, 0.0)


# ---------------------------------------------------------------------
# Helper function: compute lidar pose on circular orbit
# Chrono sensors use +X as the forward-looking direction.
# ---------------------------------------------------------------------

def get_orbiting_lidar_pose(t):
    angle = orbit_angular_speed * t

    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height

    pos = chrono.ChVector3d(x, y, z)

    # Point lidar +X axis toward the mesh center.
    # Horizontal yaw faces the origin.
    yaw = angle + math.pi

    # Pitch down toward the center.
    dist_horizontal = orbit_radius
    pitch = math.atan2(orbit_height, dist_horizontal)

    q_yaw = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
    q_pitch = chrono.QuatFromAngleAxis(pitch, chrono.ChVector3d(0, 1, 0))

    rot = q_yaw * q_pitch

    return chrono.ChFramed(pos, rot)


# ---------------------------------------------------------------------
# 1. Initialize PyChrono system
# ---------------------------------------------------------------------

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# ---------------------------------------------------------------------
# 2. Load Wavefront OBJ mesh and attach it to a fixed body
# ---------------------------------------------------------------------

if not os.path.isfile(OBJ_FILE):
    raise FileNotFoundError(
        f"OBJ file not found: {OBJ_FILE}\n"
        "Set OBJ_FILE to a valid Wavefront .obj mesh."
    )

mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(OBJ_FILE, False, True)

mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(mesh)
mesh_shape.SetName("fixed_obj_mesh")
mesh_shape.SetMutable(False)
mesh_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.65))

mesh_body = chrono.ChBody()
mesh_body.SetName("Fixed OBJ Mesh Body")
mesh_body.SetFixed(True)
mesh_body.SetPos(mesh_center)
mesh_body.AddVisualShape(mesh_shape)

system.Add(mesh_body)

# ---------------------------------------------------------------------
# Optional scene visualization using Irrlicht
# ---------------------------------------------------------------------

use_irrlicht = True

if use_irrlicht:
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Lidar Orbit Around OBJ Mesh")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(
        chrono.ChVector3d(8, -8, 5),
        chrono.ChVector3d(0, 0, 0)
    )
    vis.AddTypicalLights()
else:
    vis = None


# ---------------------------------------------------------------------
# 3. Create sensor manager
# ---------------------------------------------------------------------

manager = sens.ChSensorManager(system)

# Add lighting for the sensor rendering scene.
manager.scene.AddPointLight(
    chrono.ChVector3f(5, -5, 5),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0
)

manager.scene.AddPointLight(
    chrono.ChVector3f(-5, 5, 5),
    chrono.ChColor(1.0, 1.0, 1.0),
    300.0
)

# ---------------------------------------------------------------------
# 4. Create lidar sensor
# ---------------------------------------------------------------------

initial_lidar_pose = get_orbiting_lidar_pose(0.0)

lidar = sens.ChLidarSensor(
    mesh_body,                 # parent body
    lidar_update_rate,          # update rate [Hz]
    initial_lidar_pose,         # offset pose relative to parent body
    horizontal_samples,         # horizontal samples
    vertical_samples,           # vertical samples
    horizontal_fov,             # horizontal field of view
    vertical_fov_upper,         # max vertical angle
    vertical_fov_lower,         # min vertical angle
    max_lidar_range             # maximum range
)

lidar.SetName("Orbiting Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)

# ---------------------------------------------------------------------
# 5. Add lidar filters
# ---------------------------------------------------------------------

# Convert raw lidar depth/intensity buffer to XYZI point cloud.
lidar.PushFilter(sens.ChFilterPCfromDepth())

# Add Gaussian-style noise to XYZI data.
# Arguments are representative noise levels for x/y/z/intensity.
lidar.PushFilter(
    sens.ChFilterLidarNoiseXYZI(
        0.01,   # x/y/z position noise scale
        0.001,  # horizontal angular noise
        0.001,  # vertical angular noise
        0.01    # intensity noise
    )
)

# Visualize point cloud in a separate window.
lidar.PushFilter(
    sens.ChFilterVisualizePointCloud(
        640,
        480,
        1.0,
        "Lidar Point Cloud"
    )
)

# Save point cloud data to disk.
lidar.PushFilter(
    sens.ChFilterSavePtCloud(OUTPUT_DIR)
)

# Make XYZI point cloud accessible from Python.
lidar.PushFilter(
    sens.ChFilterXYZIAccess()
)

manager.AddSensor(lidar)


# ---------------------------------------------------------------------
# 6. Simulation loop
# ---------------------------------------------------------------------

print("Starting simulation...")
print(f"OBJ mesh: {OBJ_FILE}")
print(f"Lidar output directory: {OUTPUT_DIR}")

while system.GetChTime() < simulation_time:
    time = system.GetChTime()

    # Dynamically update lidar position in an orbit around the mesh.
    lidar_pose = get_orbiting_lidar_pose(time)
    lidar.SetOffsetPose(lidar_pose)

    # Update sensors.
    manager.Update()

    # Access and print lidar buffer data.
    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()

    if xyzi_buffer.HasData():
        xyzi = xyzi_buffer.GetXYZI()

        # xyzi is typically an N x 4 NumPy-like array:
        # columns: x, y, z, intensity
        num_points = xyzi.shape[0]

        print(f"\nTime: {time:.3f} s")
        print(f"Number of lidar points: {num_points}")

        if num_points > 0:
            print("First five XYZI points:")
            print(xyzi[:5])
    else:
        print(f"Time: {time:.3f} s | No new lidar data")

    # Render Chrono scene if Irrlicht is enabled.
    if vis is not None:
        if not vis.Run():
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Advance dynamics.
    system.DoStepDynamics(step_size)

print("Simulation complete.")