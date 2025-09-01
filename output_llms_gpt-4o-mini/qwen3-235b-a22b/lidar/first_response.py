import math
import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens

# -------------------------------
# Initialize the PyChrono system
# -------------------------------
chrono.SetChronoDataPath('path_to_data')  # Optional: Set data path if needed
system = chrono.ChSystemNSC()

# -------------------------------
# Load the mesh from a .obj file
# -------------------------------
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('path/to/mesh.obj', True, True)  # Replace with your mesh path

# Create a fixed body and attach the mesh
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# -------------------------------
# Set up Irrlicht visualization
# -------------------------------
application = irr.ChIrrApp(system, 'PyChrono Lidar Simulation', irr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()

# -------------------------------
# Initialize the sensor manager
# -------------------------------
manager = sens.ChSensorManager(system)

# -------------------------------
# Create and configure the lidar sensor
# -------------------------------
lidar = sens.ChLidarSensor(
    mesh_body,  # Parent body
    100,        # Horizontal resolution (samples per line)
    10,         # Vertical resolution (number of lines)
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  # Initial frame
    100,        # Max range
    360,        # Horizontal FOV (degrees)
    40,         # Vertical FOV (degrees)
    0.01,       # Horizontal angle step
    0.01        # Vertical angle step
)
lidar.SetName("LidarSensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.01)

# -------------------------------
# Add noise filter (Gaussian)
# -------------------------------
noise = sens.ChNoiseModelGaussian(0.0, 0.01)  # Mean 0, Std Dev 0.01
lidar.PushFilter(noise)

# -------------------------------
# Add visualization filter
# -------------------------------
lidar.PushFilter(sens.ChFilterVisualize(100, 10, "Lidar Points"))

# -------------------------------
# Add saving filter (optional)
# -------------------------------
lidar.PushFilter(sens.ChFilterSave("output/lidar_data"))

# -------------------------------
# Add the lidar to the manager
# -------------------------------
manager.AddSensor(lidar)

# -------------------------------
# Simulation loop
# -------------------------------
while application.GetDevice().run():
    time = system.GetChTime()

    # Orbital parameters
    radius = 5.0
    height = 2.0
    angular_speed = 0.5  # Radians per second
    theta = angular_speed * time

    # Compute position in orbit
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    z = height

    # Compute orientation to point toward the origin
    yaw = theta + math.pi
    quat = chrono.ChQuaternionD(chrono.ChAngleAxisD(yaw, chrono.ChVectorD(0, 0, 1)))

    # Update lidar's position and orientation relative to the mesh body
    lidar.SetParentFrame(chrono.ChFrameD(chrono.ChVectorD(x, y, z), quat))

    # Update sensor manager
    manager.Update()

    # Step the simulation
    system.DoStepDynamics(0.01)

    # Print lidar data if available
    if lidar.HasData():
        data = lidar.GetMostRecentBuffer()
        print(f"Time: {time}, Lidar data points: {len(data.point_cloud)}")

    # Render the scene
    application.BeginScene()
    application.DrawAll()
    application.EndScene()