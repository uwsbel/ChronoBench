import pychrono as pc
from pychrono import core, mesh, sensors, movement
import numpy as np

# Initialize PyChrono environment
pc.init()

# Load the triangular mesh from the .obj file
mesh_path = "triangle_mesh.obj"
mesh = pc.load_obj(mesh_path)

# Create a fixed body from the mesh
body = pc.create_fixed_body(mesh, mass=0)  # Mass is set to 0 for a fixed body

# Create a sensor manager and add a camera sensor with noise filter
sensor_manager = pc.SensorManager(body)
sensor = pc.create_camera_sensor(sensor_manager, name="orbital_camera")

# Add a Gaussian noise filter to the sensor
from pychrono.sensors import gaussian_filter
sensor.add_noise_filter(gaussian_filter)

# Set camera properties
sensor.sensor_properties.sensor_type = pc.CAMERA_TYPE PINHOLE
sensor.sensor_properties.resolution = 512  # Resolution in pixels
sensor.sensor_properties.frame_rate = 30  # Frames per second

# Create a rotational movement for the camera orbiting around the mesh
pivot_point = body.get_body_point("center")  # Get the center of the mesh as pivot
rotational_motion = pc.create_rotational_movement(pivot_point, body, name="camera_orbit")

# Set the rotation axis (let's use Z-axis for orbital motion)
rotational_motion.set_rotation_axis(pc.Vec3d(0, 0, 1))

# Set the rotation speed (degrees per second)
rotational_motion.set_rotation_speed(2)  # Adjust this value to control rotation speed

# Set simulation time step
time_step = 0.01

# Initialize simulation
simulation = pc.Simulation(body, time_step, max_time=1000, gui=True)

# Create a print function to output camera buffer data
def print_camera_data(frame):
    image = sensor.get_image()
    print(f"Frame: {frame}")
    print(f"Camera buffer data: {image}")
    print("\n")

# Set up the simulation loop
simulation.add_event("camera_orbit", callback=print_camera_data, event_type=pc.SIM_EVENT_STEP)

# Start the simulation
simulation.run()