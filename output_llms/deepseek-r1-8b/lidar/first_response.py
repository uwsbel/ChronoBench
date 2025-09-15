import pychrono as pychrono
from pychrono.objects import *
from pychrono.sensors import *
from pychrono.render import *
import numpy as np

# Initialize PyChrono environment
pychrono.init()

# Create a scene
scene = pychrono.Scene()

# Load the triangular mesh from the Wavefront .obj file
obj = pychrono.objects.MeshObject("triangular_mesh.obj")
scene.add_object(obj)

# Set the object as fixed
obj.set_body_type("fixed")

# Add a lidar sensor
lidar_sensor = pychrono.sensors.LidarSensor()
lidar_sensor.set_noise_level(0.1)  # Set noise level for lidar data
lidar_sensor.set_parent(obj)  # Attach lidar to the mesh object

# Define the orbit path for the lidar
orbit_radius = 5.0  # Radius of the orbit
orbit_vertical = 5.0  # Vertical offset from the mesh
orbit_speed = 1.0  # Speed of orbital movement
orbit_period = 10.0  # Time to complete one orbit

lidar = pychrono.objects.Body()
lidar.set_body_type("dynamic")
lidar.set_mass(0.0)  # No mass for the lidar
lidar.set_size(0.1, 0.1, 0.1)  # Small size for the lidar
lidar.set_position(obj.get_position() + pychrono.Vec3(orbit_radius, 0, orbit_vertical))
lidar.set_velocity(pychrono.Vec3(orbit_speed, 0, 0))

# Create an orbit constraint for the lidar
orbit_constraint = pychrono.constraints.OrbitConstraint(lidar, obj)
scene.add_constraint(orbit_constraint)

# Set up the camera
camera = pychrono.objects.Camera()
camera.set_position(pychrono.Vec3(10, 10, 10))
camera.set_look_at(pychrono.Vec3(0, 0, 0))
camera.set_up(pychrono.Vec3(0, 1, 0))
renderer = pychrono.render.Renderer(camera)
renderer.setClearColor(0.5, 0.5, 0.5)

# Add visualization for the lidar data
lidar_visual = pychrono.render.PointCloudVisualizer()
lidar_visual.set_source(lidar.get_source())
renderer.add(lidar_visual)

# Add visualization for the mesh
mesh_visual = pychrono.render.MeshVisualizer()
mesh_visual.set_source(obj.get_source())
renderer.add(mesh_visual)

# Simulation parameters
simulation_time = pychrono.SimTime(0, 1.0)  # 1 second
final_time = pychrono.SimTime(10, 1.0)    # 10 seconds
dt = pychrono.SimTime(0.01, 1.0)        # Time step

# Initialize the lidar buffer for saving data
lidar_buffer = []
current_time = 0.0

# Main simulation loop
while simulation_time < final_time:
    # Update simulation
    scene.update()
    
    # Update lidar position along the orbit
    current_time += dt.get_value()
    angle = (current_time / orbit_period) * 360.0
    lidar.set_position(obj.get_position() + pychrono.Vec3(
        orbit_radius * np.cos(angle * np.pi / 180),
        0,
        orbit_vertical + orbit_radius * np.sin(angle * np.pi / 180)
    ))

    # Update lidar data
    lidar_data = lidar.get_source().get_point_cloud()
    lidar_buffer.append(lidar_data.copy())
    
    # Print lidar buffer data
    print(f"Lidar Data at time {current_time:.3f}:")
    print("PointCloud Shape:", lidar_data.shape)
    print("First few points:", lidar_data[:3, :])
    
    # Next simulation step
    simulation_time += dt

# Save lidar data to file
with open("lidar_data.npy", "wb") as f:
    np.save(f, np.array(lidar_buffer))

# Close the visualization
renderer.close()