import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math

# Initialize Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create fixed triangular mesh body from .obj file
mesh_body = chrono.ChBodyEasyMesh("mesh.obj", 1000, True, True, chrono.ChVisualShapeMesh.Show_Faces)
mesh_body.SetName("Fixed Mesh Body")
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
system.Add(mesh_body)

# Create orbiting lidar body
lidar_body = chrono.ChBody()
lidar_body.SetName("Lidar Carrier")
lidar_body.SetPos(chrono.ChVector3d(5, 0, 1))  # Initial orbit position
lidar_body.SetFixed(True)  # Orbits will be manually updated
system.Add(lidar_body)

# Add sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(10, 10, 10), 1, 1, 1)
manager.scene.SetAmbientLight(chrono.ChColor(0.4, 0.4, 0.4))

# Configure lidar sensor
lidar = sens.ChLidarSensor(
    lidar_body,  # Parent body
    10,          # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)),  # Offset pose
    360,         # Horizontal samples
    1,           # Vertical samples
    -chrono.CH_PI / 2,  # Horizontal FOV (radians)
    chrono.CH_PI / 2,   # Vertical FOV (radians)
    100          # Max range (m)
)
lidar.SetName("Orbiting Lidar")
lidar.SetLidarRender(sens.ChVisualSystem.Type_LidarPoints)
lidar.SetSave(True)
lidar.SetFilename("lidar_data_{}.csv")

# Add noise model
lidar.AddNoiseModel(sens.ChLidarNoiseXYZNormal(0.01, 0.01, 0.01))

manager.AddSensor(lidar)

# Visualization setup
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Orbiting Lidar Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 8), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()

# Simulation loop parameters
time_step = 1e-3
orbit_radius = 5
orbit_speed = 0.5  # radians per second
simulation_time = 5.0

# Main simulation loop
while system.GetChTime() < simulation_time:
    # Update lidar orbit position
    angle = orbit_speed * system.GetChTime()
    lidar_body.SetPos(chrono.ChVector3d(
        orbit_radius * math.cos(angle),
        orbit_radius * math.sin(angle),
        1
    ))
    
    # Update sensor manager
    manager.Update()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Print lidar data if available
    if lidar.GetMostRecentBuffer():
        buffer = lidar.GetMostRecentBuffer().GetMetadata()
        print(f"Time {system.GetChTime():.4f}: {buffer.shape[0]} lidar points")
        if buffer.shape[0] > 0:
            print("Sample points:", buffer[:3, :3])

    vis.Render()