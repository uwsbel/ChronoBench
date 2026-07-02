import pychrono as chrono
import pychrono.sensor as sens
import numpy as np

# Initialize PyChrono environment
chrono.SetChronoDataPath('/path/to/chrono/data/')  # Update with actual path if needed
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No gravity

# Create fixed triangular mesh body
mesh_body = chrono.ChBody()
mesh_body.SetName("FixedMesh")
mesh_body.SetFixed(True)
mesh_body.EnableCollision(False)
system.Add(mesh_body)

# Add Wavefront .obj mesh visualization
mesh_shape = chrono.ChVisualShapeMesh()
mesh_shape.SetFilename("path/to/mesh.obj")  # Replace with actual .obj file path
mesh_shape.SetMutable(False)
mesh_body.AddVisualShape(mesh_shape, chrono.ChFramed())

# Create orbiting body for lidar sensor
lidar_body = chrono.ChBody()
lidar_body.SetName("LidarCarrier")
lidar_body.SetFixed(False)
lidar_body.EnableCollision(False)
system.Add(lidar_body)

# Sensor system setup
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 10), 1.0, 0.5, 0.5)
manager.Update()

# Configure lidar sensor
lidar = sens.ChLidarSensor(
    lidar_body,                  # Body to attach to
    10.0,                        # Update rate (Hz)
    chrono.ChFrameD(),          # Offset frame (default)
    200,                        # Horizontal samples
    30,                         # Vertical samples
    2 * np.pi,                  # Horizontal FOV (radians)
    0.15,                       # Vertical FOV (radians)
    10.0,                       # Max distance (m)
    0.01                        # Noise intensity
)
lidar.SetName("OrbitingLidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / 10.0)

# Add noise filter and visualization
lidar.AddNoiseModel(sens.ChLidarNoiseNone())
lidar.SetVisualizationType(sens.ChVisualizeSensor::VISUALIZE_LIDAR)
lidar.SetSaveEnabled(True)
lidar.SetFilename("lidar_data_")

manager.AddSensor(lidar)

# Simulation parameters
orbit_radius = 5.0
angular_speed = 0.5  # rad/s
time_step = 1e-3
simulation_time = 10.0

# Simulation loop
time = 0
while time < simulation_time:
    # Update lidar orbit position
    angle = angular_speed * time
    lidar_pos = chrono.ChVector3d(
        orbit_radius * np.cos(angle),
        orbit_radius * np.sin(angle),
        2.0  # Fixed height
    )
    lidar_body.SetPos(lidar_pos)
    
    # Update sensor manager
    manager.Update()
    
    # Access and print lidar buffer data
    if lidar.GetMostRecentBuffer():
        buffer = lidar.GetMostRecentBuffer().GetValue()
        if isinstance(buffer, sens.ChLidarBuffer):
            print(f"Time: {time:.4f}, Lidar Points: {buffer.GetBufferSize()}")
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    time += time_step