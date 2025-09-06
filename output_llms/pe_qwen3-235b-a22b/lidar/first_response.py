import os
import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No gravity for this simulation

# Set the Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "data"))

# Load the triangular mesh from a Wavefront .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/mesh.obj"))  # Replace with your mesh path
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetFixed(True)  # Fix the mesh in place
sys.Add(mesh_body)

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Orbiting Mesh Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Initialize sensor manager
manager = sens.ChSensorManager(sys)

# Create a mount body for the lidar
orbit_radius = 5.0
mount = chrono.ChBody()
sys.Add(mount)
mount.SetPos(chrono.ChVector3d(orbit_radius, 0, 1.0))  # Initial position above the mesh
mount.SetMass(1.0)
mount.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

# Create the lidar sensor
lidar = sens.ChLidarSensor(
    mount,                    # Body to attach the lidar
    100,                    # Horizontal samples
    10,                     # Vertical samples
    chrono.ChFrameD(),      # Offset from mount
    360,                    # Horizontal field of view (degrees)
    40,                     # Vertical field of view (degrees)
    100,                    # Max distance (meters)
    sens.LidarBeamShape_RECTANGULAR,  # Beam shape
    0.01,                   # Sample radius
    0.01,                   # Divergence angle
    sens.LidarReturnMode_STRONGEST    # Return mode
)

# Add noise filter to the lidar
lidar.PushFilter(sens.ChFilterNoiseLidar(0.01))  # Add Gaussian noise with 0.01m standard deviation

# Convert depth data to point cloud
lidar.PushFilter(sens.ChFilterPCfromDepth())

# Visualize the point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1, "Point Cloud"))

# Save point cloud data to disk
lidar.PushFilter(sens.ChFilterSavePointCloud("output/"))

# Register the lidar with the sensor manager
manager.AddSensor(lidar)

# Simulation loop
omega = 0.5  # Angular velocity (rad/s)
time_step = 0.01

while vis.Run():
    # Update lidar mount position in an orbit
    current_time = sys.GetChTime()
    theta = omega * current_time
    x = orbit_radius * math.cos(theta)
    y = orbit_radius * math.sin(theta)
    z = 1.0  # Constant height
    mount.SetPos(chrono.ChVector3d(x, y, z))

    # Advance the simulation
    sys.DoStepDynamics(time_step)

    # Update sensors
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Access and print lidar data
    buffer = lidar.GetMostRecentBuffer()
    if buffer and buffer.HasData():
        point_cloud = buffer.GetPointCloud()
        print(f"Current lidar data (first point): {point_cloud[0] if len(point_cloud) > 0 else 'No points'}")