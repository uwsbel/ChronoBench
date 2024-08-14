import pychrono as chrono
import pychrono.sensor as sens

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create the Chrono system
system = chrono.ChSystemNSC()

# Set default outward/inward shape margins for collision detection
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the ground
ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Load the mesh from the .obj file
mesh_body = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile('models/cube.obj'), # Replace with your .obj file
                                   1000,  # Density
                                   True,  # Visualization
                                   True)  # Collision
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# ---------------------------------------------------------------------
#
#  Create a sensor manager and add a camera sensor
#

# Create a sensor manager
manager = sens.ChSensorManager(system)

# Create a camera sensor
camera = sens.ChCameraSensor(
    mesh_body,       # Body to attach the sensor to
    10,              # Update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 1, 2)),  # Position relative to the body
    512,             # Image width
    512              # Image height
)
camera.SetName("Camera Sensor")
camera.SetCollectionWindow(0.1)  # Time window for data collection

# Add noise to the sensor data
# (Example: Gaussian noise)
noise = sens.ChNoiseNormal(0, 0.01)  # Mean, standard deviation
camera.AddNoise(noise)

# Add a visualization of the sensor field of view
camera.ShowSensor(True)

# Add the camera to the sensor manager
manager.AddSensor(camera)

# ---------------------------------------------------------------------
#
#  Simulation loop
#

# Simulation parameters
time_step = 0.01
time_end = 10

# Orbit parameters
orbit_radius = 3
orbit_speed = 0.5

# Simulate the system
time = 0
while time < time_end:
    # Update camera position to orbit the mesh
    angle = time * orbit_speed
    camera_pos = chrono.ChVectorD(orbit_radius * chrono.ChCos(angle), 1, orbit_radius * chrono.ChSin(angle))
    camera.GetFrame().SetPos(camera_pos)

    # Advance the simulation
    system.DoStepDynamics(time_step)
    manager.Update()

    # Get the camera buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Time: {time}, Buffer Data: {buffer.GetBuffer().shape}")

    # Increment time
    time += time_step