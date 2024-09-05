import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np

# Initialize the ChronoSystem
system = chrono.ChSystemNSC()

# Create a mesh body from an .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('path/to/your/mesh.obj', False, True)

mesh_body = chrono.ChBodyEasyMesh(
    mesh,
    1000,  # density
    True,  # visualization
    True,  # collide
    chrono.ChContactMethod_NSC,
    True  # enable boundary layer
)
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# Create a sensor manager
manager = sens.ChSensorManager(system)

# Create a lidar sensor
lidar_params = sens.ChLidarParams()
lidar_params.vertical_scanning = True
lidar_params.vertical_samples = 32
lidar_params.horizontal_samples = 1000
lidar_params.max_vert_angle = chrono.CH_C_PI / 12
lidar_params.min_vert_angle = -chrono.CH_C_PI / 12
lidar_params.max_distance = 100
lidar_params.min_distance = 0.1
lidar_params.fps = 10

lidar = sens.ChLidarSensor(
    mesh_body,  # body to which the sensor is attached
    10,  # update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),  # offset pose
    1000,  # number of horizontal samples
    32,  # number of vertical channels
    chrono.CH_C_2PI,  # horizontal field of view
    lidar_params
)

# Add noise models to the lidar
lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
lidar.PushFilter(sens.ChFilterLidarReduce())

# Add visualizations
lidar.PushFilter(sens.ChFilterVisualize(1000, 1000, "Lidar Data"))

# Add data saving
lidar.PushFilter(sens.ChFilterSave("lidar_data/"))

# Add the lidar to the sensor manager
manager.AddSensor(lidar)

# Simulation loop
sim_time = 0
orbit_radius = 5
orbit_height = 2
orbit_speed = 0.5

while sim_time < 10:  # Run for 10 seconds
    # Update lidar position in an orbit
    angle = orbit_speed * sim_time
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height
    
    new_pos = chrono.ChVectorD(x, y, z)
    new_rot = chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 0, 1))
    new_frame = chrono.ChFrameD(new_pos, new_rot)
    
    lidar.SetOffsetPose(new_frame)

    # Step the system
    system.DoStepDynamics(0.01)
    
    # Update sensors
    manager.Update()

    # Access and print lidar data
    lidar_data = lidar.GetMostRecentRawData()
    if lidar_data.HasData():
        ranges = np.frombuffer(lidar_data.ranges, dtype=np.float32)
        intensities = np.frombuffer(lidar_data.intensities, dtype=np.float32)
        print(f"Time: {sim_time:.2f}, Ranges: {ranges[:5]}, Intensities: {intensities[:5]}")

    sim_time += 0.01

print("Simulation complete")