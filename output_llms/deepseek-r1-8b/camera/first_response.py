import pychrono as pychrono
import pychrono.visualizations as pyvis
import numpy as np

# Initialize PyChrono environment
pychrono.init()

# Create a scene and add a triangular mesh from a Wavefront .obj file
scene = pychrono.Scene()
# Load the triangular mesh from the Wavefront .obj file
mesh = pychrono.Mesh("data/triangular_mesh.obj")
# Add the mesh as a fixed body to the scene
body = pychrono.Body()
body.set_mass(0)  # Make it a fixed body
body.set_name("Triangular Mesh")
scene.add_body(body, mesh)

# Add a camera sensor to the body
sensor = pychrono.Sensors.CameraSensor(body)
# Attach a sensor manager to the camera sensor
sensor_manager = pychrono.Sensors.SensorManager(sensor)
# Add a noise filter to the camera sensor
noise_filter = pychrono.Sensors.NoiseFilters.GaussianNoiseFilter(sigma=0.1)
sensor_manager.add_filter(noise_filter)

# Define the camera's orbit motion
def update_camera(sim, body, time):
    # Define the orbit parameters
    radius = 5.0  # Distance from the mesh
    angle = time * 0.5  # Angular speed
    # Update the body's local position for camera movement
    body_local_pos = body.get_local_position()
    body_local_pos.x = radius * pychrono.maths.sin(angle)
    body_local_pos.y = radius * pychrono.maths.cos(angle)
    body.set_local_position(body_local_pos)

# Set up the camera sensor properties
sensor.set_resolution(256, 256)  # Set camera resolution
sensor.set_focal_length(512.0)  # Set focal length for depth sensing
sensor.set_depth_mode(pychrono.Sensors.CameraSensor.DepthMode.PHI)  # Set depth sensing mode

# Set up the visualization
renderer = pyvis.Renderer()
renderer.set_camera(sensor)
renderer.set_antialiasing(16)  # Enable antialiasing for smoother images

# Create a simulation loop
simulation_loop = pychrono.SimulationLoop()
simulation_loop.set_timestep(0.1)  # Set simulation timestep

# Add the camera position update function
simulation_loop.add_function(update_camera, body, 0.1)

# Run the simulation
simulation_loop.start()

# Print the camera buffer data at each step
while True:
    # Get the current camera image
    image = sensor.get_image()
    # Convert the image to a numpy array for display
    img_array = np.array(image)
    print("Camera Image Data:")
    print(img_array)
    # Add a small delay to ensure the simulation runs at the correct speed
    pychrono.sleep(0.1)