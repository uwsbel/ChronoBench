import pychrono as chrono
import pychrono.sensor as sensor
import math

# Initialize the PyChrono system
system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, 0, 0))  # No gravity

# Create fixed triangular mesh body
mesh_body = chrono.ChBodyFixed()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.LoadWavefrontMesh("mesh.obj", False, False)
mesh_body.AddVisualShape(mesh_shape)
system.Add(mesh_body)

# Create orbiting lidar body
lidar_body = chrono.ChBody()
system.Add(lidar_body)

# Set up sensor manager and lidar sensor
sensor_manager = sensor.ChSensorManager(system)

# Configure lidar sensor
lidar = sensor.ChLidarSensor(
    lidar_body,  # Parent body
    0.01,  # Update interval (100 Hz)
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),  # Offset pose
    360,  # Horizontal samples
    1,    # Vertical channels (horizontal lidar)
    2 * math.pi,  # 360-degree horizontal FOV (radians)
    0.1745,       # 10-degree vertical FOV (radians)
    100            # Max range (meters)
)
lidar.SetName("Orbiting Lidar")
lidar.SetLag(0)
lidar.PushFilter(sensor.ChLidarNoiseNone())  # Add noise filter
lidar.SetVisualizationType(sensor.ChVisualizationType_CH_LIDAR_POINT_CLOUD)
sensor_manager.AddSensor(lidar)

# Simulation parameters
time_step = 0.01
end_time = 10
orbit_radius = 5
angular_speed = 0.5  # radians/second
theta = 0

# Simulation loop
time = 0
while time < end_time:
    # Update lidar position in circular orbit
    x = orbit_radius * math.cos(theta)
    z = orbit_radius * math.sin(theta)
    lidar_body.SetPos(chrono.ChVectorD(x, 0, z))
    
    # Advance system dynamics
    system.DoStepDynamics(time_step)
    
    # Update sensor manager
    sensor_manager.Update()
    
    # Retrieve and print lidar data
    if lidar.GetMostRecentBuffer():
        buffer = lidar.GetMostRecentBuffer().GetValue()
        print(f"\nLidar data at time {time:.2f}s:")
        # Example: Print buffer metadata (replace with actual data extraction)
        print(f"Buffer type: {type(buffer)}")
        print(f"Timestamp: {buffer.GetTimestamp()}")
    
    # Update orbit angle
    theta += angular_speed * time_step
    time += time_step