import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as chrono_sensor

# Create the simulation system and visualization
system = chrono.ChSystemNSC()
system.SetG(chrono.ChVectorD(0, -9.81, 0))
visual_system = chronoirr.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle("PyChrono Simulation")
visual_system.SetLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChVectorD(0, 3, 6))
visual_system.AddTypicalLights()
visual_system.AttachSystem(system)

# Load the mesh from a Wavefront .obj file
mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("mesh.obj"))

# Create a fixed body with the mesh
body = chrono.ChBody()
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetMass(10)
body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
body.SetFixed(True)
body.AddVisualShape(mesh)
system.Add(body)

# Create a sensor manager
sensor_manager = chrono_sensor.ChSensorManager()
body.AddSensorManager(sensor_manager)

# Create a lidar sensor
lidar = chrono_sensor.ChLidarSensor()
lidar.SetName("Lidar")
lidar.SetNoiseType(chrono_sensor.ChLidarNoiseType.GAUSSIAN)
lidar.SetNoiseParams(0.01, 0.01)
lidar.SetVisualizationType(chrono_sensor.ChLidarVisualizationType.POINT_CLOUD)
lidar.SetSaveData(True)
lidar.SetSaveDataPath("lidar_data")
sensor_manager.AddSensor(lidar)

# Initialize the simulation
system.Initialize()

# Simulate the system
step = 0.01
while visual_system.Run():
    # Update the lidar's position in an orbit around the mesh
    angle = math.pi / 2 * system.GetChTime()
    lidar.SetPos(chrono.ChVectorD(2 * math.cos(angle), 2 * math.sin(angle), 2))
    
    # Simulate the system
    system.DoStepDynamics(step)
    
    # Print lidar buffer data
    buffer = lidar.GetBuffer()
    print("Lidar Buffer Data:")
    for point in buffer:
        print(point)

    # Render the scene
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()