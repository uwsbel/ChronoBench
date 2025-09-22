importchrono
import math
import os

# Initialize the Chrono system
chrono.SetChronoDataPath()
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a fixed body for the triangular mesh
mesh_body = chrono.ChBody()
mesh_body.SetMass(0)  # Make it fixed
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.AddBody(mesh_body)

# Import the triangular mesh from .obj file
script_dir = os.path.dirname(__file__)
mesh_file = os.path.join(script_dir, "your_mesh.obj")
mesh = chrono.ChTriangleMesh()
chrono.AssetPython_mesh_from_file(mesh, mesh_file, True)

# Create a visual shape from the mesh
mesh_shape = chrono.ChVisualShape()
mesh_shape.SetMesh(mesh)
mesh_shape.SetMaterial(chrono.ChVisualMaterial())
mesh_body.AddVisualShape(mesh_shape)

# Add a fixed constraint to the mesh body
fixed = chrono.ChFixed()
fixed.Init(mesh_body, system.GetGround())
system.AddLink(fixed)

# Create a sensor manager
sensor_manager = chrono.ChSensorManager(system)

# Create and configure the lidar sensor
lidar = chrono.ChLidar()
lidar.SetName("lidar_sensor")
lidar.SetRange(10.0)  # Maximum range in meters
lidar.SetHorizontalAngle(-90, 90, 180)  # Horizontal FOV
lidar.SetVerticalAngle(-45, 45, 90)     # Vertical FOV
lidar.SetPosition(chrono.ChVectorD(2, 0, 2))  # Initial position relative to body

# Add noise filter
noise = chrono.ChLidarNoise()
noise.SetGaussianNoise(0.05, 0.001)
lidar.SetNoise(noise)

# Enable data saving
lidar.SetSaveData(True)
lidar.SetDataPath("lidar_data/")

# Attach lidar to the mesh body
lidar.Attach(mesh_body)
sensor_manager.AddSensor(lidar)

# Create visualization window
chrono.ChVisualizer.SetContactAssets()
visualizer = chrono.ChVisualizer()
visualizer.AttachSystem(system)
visualizer.SetWindowTitle("Lidar Simulation")
visualizer.Render()

# Simulation parameters
simulation_time = 10.0
step_size = 0.01
steps = int(simulation_time / step_size)

# Simulation loop
for i in range(steps):
    # Update lidar position in orbit
    angle = i * 2 * math.pi / steps
    lidar.SetPosition(chrono.ChVectorD(
        2 * math.cos(angle),
        0,
        2 * math.sin(angle)
    ))
    
    # Update sensor
    sensor_manager.Update()
    
    # Get lidar data
    data = lidar.GetPoints()
    print(f"Step {i}: Lidar data points: {len(data)}")
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    visualizer.Render()

# Save final state
chrono.SaveSystemState(system, "final_state.xml")