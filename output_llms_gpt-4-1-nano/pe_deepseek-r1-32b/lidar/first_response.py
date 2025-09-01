import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as chsensor
import pychrono irrlicht as chronoirr

# Initialize simulation system
sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "chrono_data"))

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Mesh Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))

# Load and create mesh body from .obj file
mesh_path = os.path.join(os.path.dirname(__file__), "mesh.obj")
mesh = chrono.ChTriangleMeshShape()
mesh.SetFilename(mesh_path)
mesh.SetScale(1.0)
mesh.SetName("mesh")

body = chrono.ChBodyEasyConvexHull(mesh.GetPoints(), True, True)
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetBodyFixed(True)
body.AddVisualShape(mesh)
sys.Add(body)

# Create sensor manager and lidar sensor
sensor_manager = chsensor.ChSensorManager(sys)
lidar = chsensor.ChLidar()
lidar.SetPosition(chrono.ChVector3d(2, 0, 2))  # Initial position
lidar.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
lidar.SetRange(50.0)
lidar.SetHorizontalAngleRange(-1.5708, 1.5708)  # 90 degrees
lidar.SetVerticalAngleRange(-0.7854, 0.7854)     # 45 degrees
lidar.SetNoise(0.01)
lidar.SetGridResolution(0.1)
lidar.SetGridDimensions(100, 100)
lidar.SetPointCloudDimensions(1024, 256)
lidar.SetSaveData(True)
lidar.SetSavePath(os.path.join(os.path.dirname(__file__), "lidar_data"))
lidar.SetSaveFormat(chsensor.ChSensor.E_SAVEFORMAT_PLY)
lidar.SetVisualize(True)
lidar.SetPointColor(chrono.ChColor(1, 0, 0))
lidar.SetGridColor(chrono.ChColor(0, 1, 0))
sensor_manager.AddSensor(lidar)

# Add grid for visualization
grid = chrono.ChBody()
grid.SetPos(chrono.ChVector3d(0, 0, 0))
grid.SetBodyFixed(True)
grid.GetVisualShape(0).SetDrawMode(chrono.ChVisualization DrawMode::CH_DRAW_MODE_WIREFRAME)
sys.Add(grid)

# Simulation parameters
simulation_time = 10.0
step_size = 0.01
lidar_radius = 5.0
lidar_angle = 0.0

# Simulation loop
while vis.Run() and sys.GetChTime() < simulation_time:
    # Update lidar position in orbit
    lidar_angle += 0.01
    lidar.SetPosition(chrono.ChVector3d(
        lidar_radius * math.cos(lidar_angle),
        lidar_radius * math.sin(lidar_angle),
        2
    ))
    
    # Update sensor manager
    sensor_manager.Update()
    
    # Run simulation step
    sys.DoStepDynamics(step_size)
    
    # Access lidar data
    lidar_data = lidar.GetGridData()
    print("Lidar buffer data at time", sys.GetChTime())
    print("Min distance:", lidar_data.min())
    print("Max distance:", lidar_data.max())
    print("------------------------")
    
    # Render visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Cleanup
vis.Close()