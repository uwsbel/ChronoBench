# =============================================================================
# PyChrono Simulation: Triangular Mesh with Orbiting Lidar Sensor
# =============================================================================

import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math
import os

# =============================================================================
# 1. Initialize the PyChrono Environment
# =============================================================================
print("Initializing PyChrono simulation environment...")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# =============================================================================
# 2. Create Fixed Body with Triangular Mesh from .obj File
# =============================================================================
print("Loading triangular mesh from Wavefront .obj file...")

# Create a fixed rigid body to host the mesh
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetMass(1.0)

# Load the triangular mesh from a Wavefront .obj file
tri_mesh = chrono.ChTriangleMeshConnected()

# Attempt to load from Chrono's built-in data path
obj_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")

# Fallback: if not found, try a local path or create a simple mesh
if not os.path.exists(obj_file):
    print(f"  Warning: '{obj_file}' not found. Trying alternative path...")
    obj_file = chrono.GetChronoDataFile("sensor/offroad/ground.obj")

if not os.path.exists(obj_file):
    print("  Warning: Default mesh not found. Creating a simple procedural mesh...")
    # Create a simple ground plane mesh procedurally
    tri_mesh.AddTriangle(
        chrono.ChVector3d(-10, -10, 0),
        chrono.ChVector3d( 10, -10, 0),
        chrono.ChVector3d( 10,  10, 0)
    )
    tri_mesh.AddTriangle(
        chrono.ChVector3d(-10, -10, 0),
        chrono.ChVector3d( 10,  10, 0),
        chrono.ChVector3d(-10,  10, 0)
    )
    # Add a simple box-like structure above the plane
    h = 2.0
    vertices = [
        chrono.ChVector3d(-2, -2, 0), chrono.ChVector3d( 2, -2, 0),
        chrono.ChVector3d( 2,  2, 0), chrono.ChVector3d(-2,  2, 0),
        chrono.ChVector3d(-2, -2, h), chrono.ChVector3d( 2, -2, h),
        chrono.ChVector3d( 2,  2, h), chrono.ChVector3d(-2,  2, h),
    ]
    faces = [(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),
             (3,0,4),(3,4,7),(4,5,6),(4,6,7)]
    for f in faces:
        tri_mesh.AddTriangle(vertices[f[0]], vertices[f[1]], vertices[f[2]])
    print("  Procedural mesh created successfully.")
else:
    tri_mesh.LoadWavefrontMesh(obj_file, True, True)
    print(f"  Mesh loaded from: {obj_file}")

# Attach visual shape (triangle mesh) to body
vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh.SetMesh(tri_mesh)
vis_mesh.SetName("SceneMesh")
vis_mesh.SetMutable(False)

# Apply a material to the mesh for visualization
mat = chrono.ChVisualMaterial()
mat.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.6))
mat.SetAmbientColor(chrono.ChColor(0.2, 0.2, 0.2))
mat.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
vis_mesh.material_list.append(mat)

mesh_body.AddVisualShape(vis_mesh)
sys.Add(mesh_body)
print("  Fixed mesh body added to simulation.")

# =============================================================================
# 3. Create the Sensor Manager
# =============================================================================
print("Setting up sensor manager...")
manager = sens.ChSensorManager(sys)

# Set the background and lighting for the scene
intensity = 1.5
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 50),
    chrono.ChColor(intensity, intensity, intensity),
    5000
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 10, 20),
    chrono.ChColor(intensity * 0.7, intensity * 0.7, intensity * 0.7),
    3000
)
manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))

# =============================================================================
# 4. Configure Lidar Sensor Parameters
# =============================================================================
print("Configuring lidar sensor parameters...")

# Timing parameters
update_rate      = 10.0              # Hz - how often the lidar fires
lag              = 0.0               # seconds - sensor lag time
collection_time  = 1.0 / update_rate # seconds - collection window

# Spatial resolution
horizontal_samples = 800    # number of horizontal samples per scan
vertical_samples   = 16     # number of vertical channels (e.g., 16-beam lidar)

# Field of view
horizontal_fov  = 2.0 * math.pi     # 360 degrees (full horizontal sweep)
max_vert_angle  =  math.pi / 12     # +15 degrees (upward)
min_vert_angle  = -math.pi / 6      # -30 degrees (downward)

# Range
max_range = 100.0  # meters

# Beam and return characteristics
beam_shape    = sens.LidarBeamShape_RECTANGULAR
sample_radius = 2
divergence_h  = 0.003  # horizontal beam divergence [rad]
divergence_v  = 0.003  # vertical beam divergence [rad]
return_mode   = sens.LidarReturnMode_STRONGEST_RETURN

# =============================================================================
# 5. Set Initial Lidar Pose (Orbit Start Position)
# =============================================================================
ORBIT_RADIUS = 12.0    # meters from mesh center
ORBIT_HEIGHT = 3.0     # meters above ground
ORBIT_SPEED  = 0.25    # radians per second

initial_angle = 0.0
init_x = ORBIT_RADIUS * math.cos(initial_angle)
init_y = ORBIT_RADIUS * math.sin(initial_angle)
init_z = ORBIT_HEIGHT

# Point the lidar toward the mesh center (rotate 180 deg around Z to face inward)
init_rot = chrono.QuatFromAngleAxis(
    initial_angle + math.pi,
    chrono.ChVector3d(0, 0, 1)
)
offset_pose = chrono.ChFramed(chrono.ChVector3d(init_x, init_y, init_z), init_rot)

# =============================================================================
# 6. Instantiate the Lidar Sensor
# =============================================================================
print("Creating lidar sensor...")
lidar = sens.ChLidarSensor(
    mesh_body,           # parent body the sensor is attached to
    update_rate,         # update rate [Hz]
    offset_pose,         # offset pose from parent body
    horizontal_samples,  # number of horizontal samples
    vertical_samples,    # number of vertical channels
    horizontal_fov,      # horizontal field of view [rad]
    max_vert_angle,      # maximum vertical angle [rad]
    min_vert_angle,      # minimum vertical angle [rad]
    max_range,           # maximum range [m]
    beam_shape,          # beam shape type
    sample_radius,       # sample radius for multi-ray
    divergence_h,        # horizontal divergence angle [rad]
    divergence_v,        # vertical divergence angle [rad]
    return_mode          # return intensity mode
)

lidar.SetName("OrbitingLidar")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# =============================================================================
# 7. Add Filters to the Lidar Sensor Pipeline
# =============================================================================
print("Attaching lidar filter pipeline...")

# --- Filter 1: Convert depth returns to point cloud (XYZI format) ---
# This must come first to generate XYZ coordinates from raw depth data
lidar.PushFilter(sens.ChFilterPCfromDepth())
print("  [1] ChFilterPCfromDepth      - Raw depth -> XYZ point cloud")

# --- Filter 2: Apply XYZI noise model ---
# Parameters: (mean, stdev_range, stdev_v_angle, stdev_h_angle)
lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.0, 0.01, 0.003, 0.003))
print("  [2] ChFilterLidarNoiseXYZI   - Gaussian noise on point cloud")

# --- Filter 3: Visualize the lidar point cloud in a window ---
# Parameters: (window_width, window_height, zoom_factor, window_name)
lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 600, 0.8, "Lidar Point Cloud"))
print("  [3] ChFilterVisualizePointCloud - Real-time point cloud visualization")

# --- Filter 4: Save point cloud data to disk ---
# Creates sequential files in the specified directory
save_dir = "output/lidar_ptcloud/"
os.makedirs(save_dir, exist_ok=True)
lidar.PushFilter(sens.ChFilterSavePtCloud(save_dir))
print(f"  [4] ChFilterSavePtCloud      - Saving to '{save_dir}'")

# --- Filter 5: Provide Python-accessible XYZI buffer ---
# Must be last in chain to access processed data
lidar.PushFilter(sens.ChFilterXYZIAccess())
print("  [5] ChFilterXYZIAccess       - Python buffer access enabled")

# Add the configured lidar to the sensor manager
manager.AddSensor(lidar)
print("  Lidar sensor added to manager.")

# =============================================================================
# 8. Simulation Loop
# =============================================================================
step_size = 1e-3     # integration time step [s]
sim_time  = 0.0      # current simulation time [s]
end_time  = 30.0     # total simulation duration [s]

step_number      = 0
total_scans      = 0
last_update_time = -1.0
lidar_period     = 1.0 / update_rate

print("\n" + "=" * 65)
print("  SIMULATION PARAMETERS")
print("=" * 65)
print(f"  Duration          : {end_time:.1f} s")
print(f"  Step size         : {step_size*1000:.1f} ms")
print(f"  Lidar update rate : {update_rate:.1f} Hz")
print(f"  Orbit radius      : {ORBIT_RADIUS:.1f} m")
print(f"  Orbit height      : {ORBIT_HEIGHT:.1f} m")
print(f"  Orbit speed       : {ORBIT_SPEED:.2f} rad/s")
print(f"  Horizontal samples: {horizontal_samples}")
print(f"  Vertical channels : {vertical_samples}")
print(f"  Max range         : {max_range:.1f} m")
print("=" * 65)
print("  Starting simulation loop...\n")

while sim_time < end_time:

    # -------------------------------------------------------------------------
    # A. Dynamically update lidar orbit position
    # -------------------------------------------------------------------------
    angle = ORBIT_SPEED * sim_time

    lidar_x = ORBIT_RADIUS * math.cos(angle)
    lidar_y = ORBIT_RADIUS * math.sin(angle)
    lidar_z = ORBIT_HEIGHT

    # Quaternion that rotates lidar to always face the mesh origin
    lidar_rot = chrono.QuatFromAngleAxis(
        angle + math.pi,
        chrono.ChVector3d(0, 0, 1)
    )

    # Update the lidar's offset pose on its parent body
    new_pose = chrono.ChFramed(
        chrono.ChVector3d(lidar_x, lidar_y, lidar_z),
        lidar_rot
    )
    lidar.SetOffsetPose(new_pose)

    # -------------------------------------------------------------------------
    # B. Update the sensor manager (triggers data acquisition)
    # -------------------------------------------------------------------------
    manager.Update()

    # -------------------------------------------------------------------------
    # C. Read and print lidar buffer data at each sensor update
    # -------------------------------------------------------------------------
    if sim_time - last_update_time >= lidar_period - (step_size * 0.5):
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()

        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            total_scans += 1

            # Buffer shape: (vertical_samples, horizontal_samples, 4)
            # Last dimension: [X, Y, Z, Intensity]
            num_channels = xyzi_data.shape[0]
            num_h_samples = xyzi_data.shape[1]
            total_points = num_channels * num_h_samples

            # Count valid (non-zero range) returns
            ranges = np.sqrt(
                xyzi_data[:, :, 0]**2 +
                xyzi_data[:, :, 1]**2 +
                xyzi_data[:, :, 2]**2
            )
            valid_mask  = ranges > 0.01
            valid_count = int(np.sum(valid_mask))
            avg_range   = float(np.mean(ranges[valid_mask])) if valid_count > 0 else 0.0
            avg_intensity = float(np.mean(xyzi_data[:, :, 3][valid_mask])) if valid_count > 0 else 0.0

            # Find a sample point (median valid return)
            sample_str = "N/A"
            if valid_count > 0:
                valid_indices = np.argwhere(valid_mask)
                mid_idx = valid_indices[len(valid_indices) // 2]
                pt = xyzi_data[mid_idx[0], mid_idx[1], :]
                sample_str = f"[{pt[0]:+.3f}, {pt[1]:+.3f}, {pt[2]:+.3f}, i={pt[3]:.3f}]"

            print(f"[t={sim_time:6.3f}s | Scan #{total_scans:4d}]")
            print(f"  Lidar position  : ({lidar_x:+7.3f}, {lidar_y:+7.3f}, {lidar_z:+5.3f}) m")
            print(f"  Orbit angle     : {math.degrees(angle):6.1f} deg")
            print(f"  Buffer shape    : {xyzi_data.shape}  "
                  f"[channels x samples x (X,Y,Z,I)]")
            print(f"  Total points    : {total_points:6d} | "
                  f"Valid hits: {valid_count:6d} ({100*valid_count/total_points:.1f}%)")
            print(f"  Avg range       : {avg_range:.3f} m | "
                  f"Avg intensity: {avg_intensity:.4f}")
            print(f"  Sample point    : {sample_str}")
            print()

            last_update_time = sim_time

        else:
            # Buffer not yet populated (typical in first frames)
            print(f"[t={sim_time:6.3f}s] Waiting for lidar buffer to populate...")
            last_update_time = sim_time

    # -------------------------------------------------------------------------
    # D. Advance the simulation by one time step
    # -------------------------------------------------------------------------
    sys.DoStepDynamics(step_size)
    sim_time  += step_size
    step_number += 1

# =============================================================================
# 9. Simulation Complete
# =============================================================================
print("=" * 65)
print("  SIMULATION COMPLETE")
print("=" * 65)
print(f"  Total simulation time : {end_time:.1f} s")
print(f"  Total steps executed  : {step_number}")
print(f"  Total lidar scans     : {total_scans}")
print(f"  Point cloud data saved to: '{save_dir}'")
print("=" * 65)