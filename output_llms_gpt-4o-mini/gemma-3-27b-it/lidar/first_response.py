import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensor

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("../chrono/data/")

# ---------------------------------------------------------------------
#
#  CREATE THE SIMULATION SYSTEM
#
# ---------------------------------------------------------------------

my_system = chrono.ChSystemNSC()
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
#
#  CREATE THE PHYSICAL OBJECTS
#
# ---------------------------------------------------------------------

# Load the mesh from a Wavefront .obj file
mesh_file = "cube.obj"  # Replace with your .obj file
mesh_path = os.path.join(chrono.GetChronoDataPath(), mesh_file)

body = chrono.ChBody()
body.SetBodyFixed(True)

# Add a mesh shape to the body
mesh_shape = chrono.ChVisualShapeFile()
mesh_shape.SetFile(mesh_path)
body.AddVisualShape(mesh_shape)
body.AddCollisionShape(mesh_shape)
body.SetMass(1.0)
body.SetPos(chrono.ChVector3d(0, 0, 0))

my_system.Add(body)

# ---------------------------------------------------------------------
#
#  CREATE THE LIDAR SENSOR
#
# ---------------------------------------------------------------------

# Create a sensor manager
sensor_manager = sensor.ChSensorManager(my_system)

# Create a lidar sensor
lidar = sensor.ChLidarSensor()
lidar.SetBody(body)
lidar.SetScanRate(30.0)  # Scans per second
lidar.SetRange(10.0)  # Maximum range in meters
lidar.SetResolution(0.1)  # Angular resolution in radians
lidar.SetNoiseStdDev(0.01)  # Standard deviation of noise
lidar.SetVisualize(True)  # Visualize the lidar rays
lidar.SetSaveData(True)  # Save lidar data to file
lidar.SetSaveDataFrequency(1.0)  # Save data every 1 second

# Add the lidar sensor to the sensor manager
sensor_manager.AddSensor(lidar)

# ---------------------------------------------------------------------
#
#  CREATE THE VISUALIZATION SYSTEM
#
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Lidar Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(5, 5, -5))
vis.AddTypicalLights()
vis.AddSkyBox()

# ---------------------------------------------------------------------
#
#  SIMULATION LOOP
#
# ---------------------------------------------------------------------

time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update lidar position (orbit around the mesh)
    time = my_system.GetChTime()
    orbit_radius = 2.0
    lidar_position = chrono.ChVector3d(
        orbit_radius * math.cos(time),
        0.0,
        orbit_radius * math.sin(time)
    )
    lidar.SetOffset(lidar_position)

    # Do a simulation step
    my_system.DoStepDynamics(time_step)

    # Print lidar buffer data
    if lidar.HasNewData():
        lidar_data = lidar.GetBufferData()
        print("Lidar Data (Number of points:", len(lidar_data), "):")
        # Example: Print the first few points
        for i in range(min(5, len(lidar_data))):
            print(f"  Point {i+1}: {lidar_data[i]}")