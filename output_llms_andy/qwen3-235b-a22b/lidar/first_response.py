import math
import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irrlicht

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# Load the triangular mesh from .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('path/to/your/mesh.obj', True, True)  # Update with actual path
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# Create lidar sensor body
lidar_body = chrono.ChBody()
lidar_body.SetPos(chrono.ChVectorD(2, 0, 0.5))  # Initial position
system.Add(lidar_body)

# Initialize sensor manager
manager = sens.ChSensorManager(system)

# Create lidar sensor with noise and visualization
lidar = sens.ChLidarSensor(
    lidar_body,              # Attach to lidar body
    10,                      # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),  # Offset
    360,                     # Horizontal resolution
    1,                       # Vertical resolution
    chrono.CH_C_PI,          # Horizontal FOV
    -chrono.CH_C_PI / 12,    # Vertical FOV start
    chrono.CH_C_PI / 12      # Vertical FOV end
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0.01)

# Add Gaussian noise to lidar data
noise = sens.ChNoiseGaussian(0.0, 0.01)  # Mean 0, std 0.01
lidar.AddNoise(noise)

# Add visualization for lidar point cloud
vis = sens.ChSensorVisualizer()
lidar.AddVisualizer(vis)

manager.AddSensor(lidar)

# Set up Irrlicht visualization
application = irrlicht.ChIrrApp(system, 'PyChrono Lidar Simulation', irrlicht.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, 3))
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation parameters
orbit_radius = 2.0
orbit_height = 0.5
angular_velocity = 0.5  # rad/s
step_size = 0.01
application.SetTimestep(step_size)

# Simulation loop
while application.GetDevice().run():
    # Update lidar orbital position
    time = system.GetChTime()
    angle = angular_velocity * time
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height
    lidar_body.SetPos(chrono.ChVectorD(x, y, z))
    
    # Rotate lidar to face mesh center
    center = chrono.ChVectorD(0, 0, 0)
    look_dir = center - lidar_body.GetPos()
    look_dir.z = 0  # Constrain to XY plane
    look_dir.Normalize()
    rot = chrono.Q_from_2_Vectors(chrono.ChVectorD(1, 0, 0), look_dir)
    lidar_body.SetRot(rot)
    
    # Step simulation
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(step_size)
    manager.Update()
    
    # Print lidar data
    buffer = lidar.GetMostRecentBuffer()
    if buffer and buffer.HasData():
        points = buffer.GetPoints()
        print(f"Time: {time:.2f}s, First point: {points[0]}")
    
    application.EndScene()