import pychrono as pc
import numpy as np

# Initialize PyChrono environment
pc.init()

# Load the triangular mesh from the Wavefront .obj file
mesh = pc.load_obj("triangle.obj")

# Initialize the mesh as a fixed body
mesh.set_body_type(pc.BodyType.FIXED)

# Create a camera sensor
sensor = pc.SensorManager()
sensor_body = mesh.add_sensor(sensor)

# Configure the camera sensor with noise filter
sensor.camera_noise_level = 0.1  # Reduce noise
sensor.contrastEnhancement = 1.2  # Adjust contrast
sensor.buffer_size = 512  # Set buffer size

# Create and position the camera
camera = pc.Camera()
camera.position = np.array([5, 5, 5])
camera.look_at = np.array([0, 0, 0])
camera.orbit_radius = 5  # Orbit radius
camera.orbit_speed = 0.5  # Orbit speed in degrees per step

# Add the camera to the sensor manager
sensor_body.add_sensor_camera(sensor, camera)

# Set up simulation
sim = pc.Simulation()
sim.set_time_step(0.1)  # Time step
sim.set_max_steps(1000)  # Maximum steps

# Print camera buffer data at each step
for step in range(sim.get_steps()):
    sim.step()
    sensor_data = sensor.get_image()
    print(f"Sensor Data at step {step}:")
    print(sensor_data)

# Run the simulation
sim.run()