import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import numpy as np
import os

# 1. Initialize PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # Zero gravity for static scene

# 2. Create fixed mesh body from OBJ file
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_path = chrono.GetChronoDataFile("models/cube.obj")  # Replace with your OBJ path
mesh_visual = chrono.ChVisualShapeModelFile(mesh_path)
mesh_body.AddVisualShape(mesh_visual)
system.Add(mesh_body)

# 3. Create sensor manager
manager = sens.ChSensorManager(system)

# 4. Configure lidar sensor
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0))  # Relative to parent body
lidar = sens.ChLidarSensor(
    parent=mesh_body,         # Attached to mesh body
    update_rate=10,           # Hz
    offset_pose=offset_pose,
    horizontal_samples=90,    # Points per scan line
    vertical_samples=5,       # Vertical scan lines
    horizontal_fov=90,        # Degrees
    vertical_fov=10,          # Degrees
    max_vertical_angle=5,     # Degrees
    min_vertical_angle=-5,    # Degrees
    max_distance=100          # Meters
)

# Add noise model
noise_model = sens.ChNoiseNormal(mean=0, stdev=0.01)
lidar.SetNoiseModel(noise_model)

# Add filter for saving data
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterAccess())
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))

# Add visualization
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0))

# Add sensor to manager
manager.AddSensor(lidar)

# 5. Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 2))
vis.AddTypicalLights()

# 6. Simulation loop parameters
orbit_radius = 3.0
orbit_height = 1.0
angular_speed = 0.1  # rad/s
time_step = 0.01
current_time = 0.0

print("Starting simulation. Press ESC to exit...")

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update lidar position in orbit
    angle = angular_speed * current_time
    x = orbit_radius * np.cos(angle)
    y = orbit_radius * np.sin(angle)
    lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(x, y, orbit_height)))

    # Update sensors
    manager.Update()

    # Print latest lidar buffer data
    buffer = lidar.GetMostRecentBuffer()
    if buffer.HasData():
        point_cloud = buffer.GetPointCloud()
        print(f"Time: {current_time:.2f}s | Points: {len(point_cloud)}")
        if len(point_cloud) > 0:
            print(f"Sample point: {point_cloud[0]}")

    # Advance simulation
    system.DoStepDynamics(time_step)
    current_time += time_step