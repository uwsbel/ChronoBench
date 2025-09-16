import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math

# Initialize Chrono system and visualization
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Lidar Mesh Simulation')
vis.Initialize()
sys.SetVisualSystem(vis)

# Load triangular mesh from .obj file (replace with actual path)
mesh_path = "path/to/mesh.obj"
mesh = chrono.ChTriangleMesh()
if not mesh.LoadWavefrontMesh(mesh_path):
    print("Error: Mesh file not found.")
    exit(1)

# Create fixed body for the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
body.AddAsset(mesh_shape)
body.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))  # Gray color
sys.Add(body)

# Setup sensor manager and lidar
sensor_mgr = sens.ChSensorManager(sys)
lidar = sens.ChLidar()
lidar.SetName("lidar_sensor")
lidar.SetPosition(chrono.ChVectorD(0, 0, 5))  # Initial position above mesh
lidar.SetRotation(chrono.Q_from_Euler_123(0, 0, 0))  # Initial orientation

# Configure lidar parameters
lidar.SetHorizontalFOV(chrono.CH_C_PI / 2)  # 90 degrees
lidar.SetVerticalFOV(chrono.CH_C_PI / 4)    # 45 degrees
lidar.SetRange(50)  # Max range in meters
lidar.SetNoiseModel(sens.ChLidarNoiseGaussian(0.05))  # 5% noise
lidar.SetNumberOfHorizontalBeams(360)
lidar.SetNumberOfVerticalBeams(1)
lidar.SetRenderPoints(True)  # Visualize points

# Add sensor to manager and enable visualization
sensor_mgr.AddSensor(lidar)
lidar.GetVisualizationAssets().Append(chrono.ChVisualizationLidar())

# Orbit parameters
orbit_radius = 10.0  # meters
angular_speed = 0.1  # radians per second
current_angle = 0.0
step_size = 0.01  # Simulation step size

# Simulation loop
while vis.Run():
    time = sys.GetChTime()
    
    # Update lidar position and orientation in orbit
    current_angle += angular_speed * step_size
    x = orbit_radius * math.cos(current_angle)
    y = orbit_radius * math.sin(current_angle)
    z = 5.0  # Height above ground
    
    lidar.SetPosition(chrono.ChVectorD(x, y, z))
    
    # Compute orientation to face the mesh (origin)
    lookat = chrono.ChVectorD(0, 0, 0)
    eye = chrono.ChVectorD(x, y, z)
    up = chrono.ChVectorD(0, 1, 0)  # Y-axis up
    forward = lookat - eye
    forward.Normalize()
    
    right = forward.Cross(up)
    right.Normalize()
    up = right.Cross(forward)
    
    rot_matrix = chrono.ChMatrix33D()
    rot_matrix.Set_A_Xaxis(right)
    rot_matrix.Set_A_Yaxis(up)
    rot_matrix.Set_A_Zaxis(forward)
    rotation = chrono.Q_from_Matrix(rot_matrix)
    lidar.SetRotation(rotation)
    
    # Update sensor and retrieve data
    sensor_mgr.Update()
    buffer = lidar.GetBuffer()
    print(f"Time {time:.2f}s: Detected {len(buffer)} points")
    
    # Advance simulation step
    sys.DoStepDynamics(step_size)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()