import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()

# 2. Add the triangular mesh as a fixed body
mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)

# Load the mesh from a Wavefront OBJ file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj")  # <-- Replace with your .obj file path
mesh.RepairDuplicateVertexes(1e-9)

# Attach the mesh as a visualization asset
mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetName("tri_mesh")
mesh_asset.SetColor(chrono.ChColor(0.4, 0.4, 0.8))
mesh_body.AddAsset(mesh_asset)

# Add the mesh body to the system
system.Add(mesh_body)

# 3. Set up the sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(2, 2, 2), chrono.ChColor(1, 1, 1), 500)

# 4. Add a lidar sensor to the mesh body
# Lidar parameters
lidar_update_rate = 10.0  # Hz
lidar_h_fov = chrono.CH_C_PI  # 180 deg
lidar_v_fov = chrono.CH_C_PI / 4  # 45 deg
lidar_max_vert_angle = lidar_v_fov / 2
lidar_min_vert_angle = -lidar_v_fov / 2
lidar_max_horz_angle = lidar_h_fov / 2
lidar_min_horz_angle = -lidar_h_fov / 2
lidar_vert_samples = 16
lidar_horz_samples = 64
lidar_max_distance = 10.0  # meters

# Initial lidar position (will be updated dynamically)
lidar_offset = chrono.ChFrameD(chrono.ChVectorD(2, 0, 1), chrono.QUNIT)

lidar = sens.ChLidarSensor(
    mesh_body,          # attached to mesh body
    lidar_update_rate,  # update rate in Hz
    lidar_offset,       # initial offset pose
    lidar_horz_samples, # horizontal samples
    lidar_vert_samples, # vertical samples
    lidar_min_horz_angle,
    lidar_max_horz_angle,
    lidar_min_vert_angle,
    lidar_max_vert_angle,
    lidar_max_distance
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0)

# Add noise filter
lidar.PushFilter(sens.ChFilterLidarNoise(0.01))  # 1cm stddev

# Add visualization filter
lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Visualization"))

# Add data saving filter
lidar.PushFilter(sens.ChFilterSaveLidarPCD("lidar_output/"))

# Add the lidar to the manager
manager.AddSensor(lidar)

# 5. Simulation loop with dynamic lidar position update
step_size = 1.0 / 100.0  # 100 Hz physics
end_time = 2.0           # seconds
orbit_radius = 2.0
orbit_height = 1.0

print("Starting simulation...")

while system.GetChTime() < end_time:
    # Orbit the lidar around the mesh
    t = system.GetChTime()
    angle = 2 * math.pi * t / end_time  # one full orbit over simulation
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height

    # Lidar looks at the origin (assuming mesh at origin)
    pos = chrono.ChVectorD(x, y, z)
    target = chrono.ChVectorD(0, 0, 0)
    up = chrono.ChVectorD(0, 0, 1)
    dir = (target - pos).GetNormalized()
    # Compute quaternion to look at the origin
    rot = chrono.Q_from_AngAxis(math.atan2(y, x), chrono.ChVectorD(0, 0, 1))

    # Update lidar's offset frame
    lidar.SetOffsetPose(chrono.ChFrameD(pos, rot))

    # Advance simulation
    system.DoStepDynamics(step_size)
    manager.Update()

    # Access lidar buffer data
    lidar_data = lidar.GetMostRecentBuffer()
    if lidar_data is not None:
        # lidar_data is a ChLidarSensorBuffer
        points = lidar_data.GetPointCloud()
        print(f"Time {t:.2f}s: Lidar points = {points.shape[0] if hasattr(points, 'shape') else len(points)}")
    else:
        print(f"Time {t:.2f}s: No lidar data yet.")

print("Simulation complete.")