import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensor

# Initialize the PyChrono system
sys = chrono.ChSystemNSC()

# Set the path to Chrono's data directory
chrono.SetChronoDataPath('/path/to/data')  # Update this to your data path

# Load a triangular mesh from a Wavefront .obj file
mesh_file_path = chrono.GetChronoDataFile('your_mesh_file.obj')  # Replace with your .obj file
mesh_body = chrono.ChBodyEasyMesh(mesh_file_path, 1000, True, True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Position the mesh at the origin
mesh_body.SetFixed(True)  # Fix the mesh body in place
sys.Add(mesh_body)

# Create a visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Triangular Mesh with Camera Sensor')
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  # Initial camera position
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Set up the camera sensor
camera = sensor.ChCameraSensor(mesh_body,  # Attach to the mesh body
                                chrono.ChFrameD(chrono.ChVectorD(0, 5, 10), chrono.Q_from_AngZ(0)),  # Camera frame
                                640, 480,  # Image width and height
                                30,  # Field of view
                                0.01, 0.1,  # Near and far plane
                                sensor.ChCameraSensor.CAMERATYPE.PERSPECTIVE)  # Camera type
camera.SetNoise(0.05)  # Set noise filter level
sys.Add(camera)

# Simulation parameters
time_step = 0.01
total_time = 5.0
num_steps = int(total_time / time_step)

# Simulation loop
for step in range(num_steps):
    # Update camera position to orbit around the mesh
    angle = step * (2 * math.pi / num_steps)  # Full orbit
    camera.SetFrame(chrono.ChFrameD(chrono.ChVectorD(5 * math.cos(angle), 5, 5 * math.sin(angle)), chrono.Q_from_AngY(angle)))

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Print camera buffer data
    camera_buffer = camera.GetImageBuffer()
    print(f"Step {step}: Camera buffer data (size: {len(camera_buffer)} bytes)")

    # Advance the simulation
    sys.DoStepDynamics(time_step)

# Cleanup
vis.Clear()