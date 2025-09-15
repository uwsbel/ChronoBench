importchrono
import math
import numpy as np
import os

# Initialize Chrono system
chrono.SetChronoDataPath()
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create fixed body with triangular mesh
mesh_filename = "your_mesh.obj"  # Replace with your .obj file path
if not os.path.exists(mesh_filename):
    raise FileNotFoundError(f"Mesh file {mesh_filename} not found")

mesh = chrono.ChTriangleMesh()
mesh.SetFilename(mesh_filename)
mesh.SetName("mesh_body")
mesh.SetPos(chrono.ChVectorD(0, 0, 0))
mesh.SetBodyFixed(True)
system.Add(mesh)

# Create camera sensor
camera_body = chrono.ChBody()
camera_body.SetBodyFixed(True)
camera_body.SetPos(chrono.ChVectorD(5, 0, 0))  # Initial position
camera_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # Initial orientation

sensor_manager = chrono.ChSensorManager()
sensor_manager.AttachTo(camera_body)

# Configure camera properties
camera = chrono.ChCameraSensor()
camera.SetName("main_camera")
camera.SetResolution(640, 480)
camera.SetPosition(chrono.ChVectorD(0, 0, 0))  # Relative to camera_body
camera.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))  # Looking along negative Z-axis
camera.SetFOV(60)
camera.SetNearPlane(0.1)
camera.SetFarPlane(100)
sensor_manager.AddSensor(camera)

# Add noise filter
def apply_noise(buffer):
    noise = np.random.normal(0, 0.02, buffer.shape)
    return np.clip(buffer + noise, 0, 1)

# Visualization
viewer = chrono.ChVisualizationWindow()
viewer.SetWindowTitle("Mesh with Camera Sensor")
viewer.AttachSystem(system)
viewer.AddVisualization(mesh)
viewer.AddVisualization(camera_body)

# Add camera field of view visualization
fov_shape = chrono.ChVisualShape()
fov_shape.SetPosition(camera_body.GetPos())
fov_shape.SetRotation(camera_body.GetRot())
fov_shape.SetScale(chrono.ChVectorD(1, 1, 1))
fov_shape.SetType(chrono.ChVisualShapeType.SPHERES)
camera_body.AddVisualShape(fov_shape)

# Simulation parameters
simulation_time = 10.0
step_size = 0.001
num_steps = int(simulation_time / step_size)

# Simulation loop
for step in range(num_steps):
    # Update camera position in orbit
    angle = step * 0.02  # Increment angle each step
    camera_body.SetPos(chrono.ChVectorD(5 * math.cos(angle), 0, 5 * math.sin(angle)))
    
    # Update sensor manager
    sensor_manager.Update()
    
    # Get camera buffer
    buffer = camera.GetBuffer()
    noisy_buffer = apply_noise(buffer)
    
    # Print buffer data
    print(f"Time: {step * step_size:.3f}s")
    for row in noisy_buffer:
        print(' '.join(map(str, row)))
    print("\n")
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    viewer.Render()

# Cleanup
viewer.Close()
chrono.ChExit()