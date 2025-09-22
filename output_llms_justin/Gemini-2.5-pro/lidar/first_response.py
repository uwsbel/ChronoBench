import pychrono as chrono
import pychrono.sensor as sens
import os
import math
import numpy as np # For potential data manipulation, though not strictly needed for printing

# ----------------------------
# Create the PyChrono system
# ----------------------------
print("Creating Chrono system...")
physics_system = chrono.ChSystemNSC()
physics_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0)) # Gravity, though body is fixed

# ------------------------------------
# Create the triangular mesh body
# ------------------------------------
print("Creating mesh body...")
# Create a fixed body
mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0)) # Position at origin
physics_system.Add(mesh_body)

# Load the .obj mesh
obj_filename = "simple_pyramid.obj"

# Check if the .obj file exists, if not create it
if not os.path.exists(obj_filename):
    print(f"Warning: '{obj_filename}' not found. Creating a dummy file.")
    with open(obj_filename, "w") as f:
        f.write("# Simple Pyramid\n")
        f.write("v 0.0 1.0 0.0\n")
        f.write("v -0.5 0.0 -0.5\n")
        f.write("v 0.5 0.0 -0.5\n")
        f.write("v 0.5 0.0 0.5\n")
        f.write("v -0.5 0.0 0.5\n")
        f.write("f 1 2 3\n")
        f.write("f 1 3 4\n")
        f.write("f 1 4 5\n")
        f.write("f 1 5 2\n")
        f.write("f 2 5 4\n")
        f.write("f 2 4 3\n")

# Create the visual shape from the mesh
# Note: For Chrono::Sensor, visual shapes are what sensors "see"
trimesh_shape = chrono.ChTriangleMeshConnected()
try:
    trimesh_shape.LoadWavefrontMesh(obj_filename, False, True) # path, load normals, load uvs
    trimesh_shape.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1)) # Position and scale if needed
    
    # Add visual shape for rendering AND sensing
    vis_mesh = chrono.ChVisualShapeTriangleMesh()
    vis_mesh.SetMesh(trimesh_shape)
    vis_mesh.SetName("PyramidMesh")
    mesh_body.AddVisualShape(vis_mesh)

    # Optional: Add collision shape if you want physics interactions later
    # For this specific request (lidar on fixed body), collision shape isn't strictly needed
    # material = chrono.ChMaterialSurfaceNSC() # Or SMC
    # coll_mesh = chrono.ChCollisionShapeTriangleMesh(material, trimesh_shape, True, False, 0.0)
    # mesh_body.AddCollisionShape(coll_mesh)
    # mesh_body.EnableCollision(True)

except Exception as e:
    print(f"Error loading OBJ file: {e}")
    print("Please ensure 'simple_pyramid.obj' exists and is valid.")
    exit()


# -----------------------------
# Create a sensor manager
# -----------------------------
print("Creating sensor manager...")
sensor_manager = sens.ChSensorManager(physics_system)
sensor_manager.SetVerbose(False) # Set to True for more detailed sensor messages

# ----------------------------------------------------
# Create Lidar sensor and add it to the sensor manager
# ----------------------------------------------------
print("Creating Lidar sensor...")
# Lidar parameters
update_rate = 10.0  # Hz, sensor update frequency
hfov = math.pi / 3  # Horizontal FOV (60 degrees)
vfov = math.pi / 6  # Vertical FOV (30 degrees)
h_samples = 120      # Number of horizontal samples
v_samples = 60       # Number of vertical samples
max_distance = 50.0  # Max detection distance

# Initial offset pose for the lidar (relative to mesh_body)
# Let's start it above the pyramid, looking down.
initial_lidar_offset_pos = chrono.ChVectorD(0, 2.5, 0)
initial_lidar_offset_rot = chrono.Q_from_AngX(-math.pi / 2) # Look downwards
initial_lidar_offset_pose = chrono.ChFrameD(initial_lidar_offset_pos, initial_lidar_offset_rot)

lidar = sens.ChLidarSensor(
    mesh_body,               # Parent body
    update_rate,             # Update rate
    initial_lidar_offset_pose, # Offset pose
    h_samples,               # Horizontal samples
    v_samples,               # Vertical samples
    hfov,                    # Horizontal FOV
    vfov,                    # Vertical FOV (max_angle - min_angle for vertical scan)
    max_distance,            # Max distance
    # sens.LidarBeamShape.RECTANGULAR, # Beam shape (default is RECTANGULAR)
    # 0,                       # Clip near (default 0)
    # sens.LidarReturnMode.STRONGEST_RETURN # Return mode (default)
)
lidar.SetName("MyLidar")
lidar.SetLag(0) # No lag
lidar.SetCollectionWindow(0) # Collect data instantaneously

# --- Add Filters ---
# 1. Noise filter: Add Gaussian noise to XYZI data
#    Parameters: mean_x, stddep_x, mean_y, stddep_y, mean_z, stddep_z, mean_i, stddep_i
noise_model_xyzi = sens.ChNoiseNormalXYZI(0.0, 0.01, 0.0, 0.01, 0.0, 0.01, 0.0, 0.0) # Noise on position, not intensity
lidar.PushFilter(noise_model_xyzi)

# 2. Visualization filter: Display lidar point cloud in a window
#    This requires an OpenGL context, usually available with Chrono::Sensor
lidar.PushFilter(sens.ChFilterVisualize("Lidar Point Cloud", 640, 480))

# 3. Save filter: Save lidar data
#    Make sure the directory exists
output_dir = "sensor_output/lidar_data/"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
lidar.PushFilter(sens.ChFilterSave(output_dir)) # Saves data in CSV format

sensor_manager.AddSensor(lidar)
print("Lidar sensor added with filters.")

# ---------------------
# Simulation loop
# ---------------------
simulation_time = 10.0  # seconds
time_step = 1.0 / update_rate # Synchronize physics step with sensor update for this demo
                               # Or use a smaller physics_time_step if dynamics are complex

current_time = 0.0

# Orbit parameters for lidar
orbit_radius = 3.0
orbit_speed = math.pi / 5  # Radians per second
orbit_height = 1.5 # Constant height for the orbit

print(f"\nStarting simulation for {simulation_time} seconds...")
print(f"Lidar will orbit the mesh at radius {orbit_radius}m, height {orbit_height}m, speed {orbit_speed:.2f} rad/s.")
print("Press Ctrl+C in the terminal to stop early if visualization window is not responsive to closing.")

try:
    while current_time < simulation_time:
        # Update Lidar position dynamically in an orbit
        angle = orbit_speed * current_time
        new_x = orbit_radius * math.cos(angle)
        new_z = orbit_radius * math.sin(angle)
        
        # New position for the lidar, relative to the mesh_body (which is at origin)
        current_lidar_pos = chrono.ChVectorD(new_x, orbit_height, new_z)
        
        # Make the lidar always point towards the origin (where the pyramid base center is)
        # Calculate direction vector from lidar to origin
        direction_to_origin = (chrono.ChVectorD(0,0.5,0) - current_lidar_pos).GetNormalized() # Point to pyramid center approx
        
        # Calculate orientation (this is a bit more involved to get it right)
        # Z-axis of lidar should be 'direction_to_origin'
        # X-axis can be 'world Y' cross 'Z-axis'
        # Y-axis is 'Z-axis' cross 'X-axis'
        lidar_z_axis = direction_to_origin
        lidar_x_axis = chrono.Vcross(chrono.ChVectorD(0,1,0), lidar_z_axis).GetNormalized()
        lidar_y_axis = chrono.Vcross(lidar_z_axis, lidar_x_axis).GetNormalized()
        
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.SetCol(0, lidar_x_axis)
        rot_matrix.SetCol(1, lidar_y_axis)
        rot_matrix.SetCol(2, lidar_z_axis)
        
        current_lidar_rot = rot_matrix.GetQuaternion()
        
        new_offset_pose = chrono.ChFrameD(current_lidar_pos, current_lidar_rot)
        lidar.SetOffsetPose(new_offset_pose)

        # Update sensor manager (collects data from sensors)
        sensor_manager.Update()

        # Advance simulation
        physics_system.DoStepDynamics(time_step)
        
        # Get and print Lidar data
        # For Lidar, common buffers are XYZI (x,y,z,intensity) or DFull (distance, vert_angle, horiz_angle, intensity)
        # Let's get XYZI
        xyzi_buffer = lidar.GetMostRecentBufferXYZI()
        if xyzi_buffer.HasData():
            # Access data on host (CPU)
            # This returns a list of lists: [[x1,y1,z1,i1], [x2,y2,z2,i2], ...]
            point_cloud_data = xyzi_buffer.GetHostBufferXYZI() 
            num_points = len(point_cloud_data)
            print(f"Time: {current_time:.2f}s - Lidar captured {num_points} points.")
            
            # Example: print first few points
            # for i in range(min(3, num_points)):
            #     print(f"  Point {i}: {point_cloud_data[i]}")
        else:
            print(f"Time: {current_time:.2f}s - No new Lidar data.")

        current_time += time_step

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")
finally:
    print("\nSimulation finished.")
    # Sensor visualization windows might stay open. Closing them manually might be needed.
    # Or, if you had an Irrlicht visualization, you'd close it here.

    # To ensure sensor data files are flushed if using ChFilterSave:
    # You can manually call SaveAll عاش عاش عاش(this is not a standard chrono function,
    # it would be a custom method if you aggregated save filters)
    # or rely on the sensor manager's destruction to handle it.
    # For ChFilterSave, data is typically saved per-update or when buffers fill.
    print(f"Lidar data saved in: {output_dir}")