import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math

# ---------------------------------------------------------------------
#  Create a Chrono system and set simulation parameters
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Simulation time step
time_step = 1e-3
system.SetStep(time_step)

# ---------------------------------------------------------------------
#  Create the simulation environment (add ground and asset)
# ---------------------------------------------------------------------
chrono.SetChronoDataPath("./data/")

# Create a body using the Wavefront .obj mesh
mesh_asset = chrono.ChAsset()
mesh_asset.AppendAsset(chrono.ChObjShapeFile("./sensor_test_assets/house.obj"))
mesh_body = chrono.ChBodyEasyMesh(
    mesh_asset,  # Mesh asset
    1000,        # Density
    True,        # Compute mass inertia
    True,        # Use visualization asset
    False        # Use collision shape
)
system.Add(mesh_body)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)  # Fix the mesh in space

# ---------------------------------------------------------------------
#  Create a sensor manager and add a lidar sensor
# ---------------------------------------------------------------------
manager = sens.ChSensorManager(system)

# Create a Lidar sensor
lidar = sens.ChLidarSensor(
    mesh_body,              # Body to attach the sensor to
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2)),  # Offset respect to body
    1.0,                   # Update rate in Hz
    chrono.ChVectorD(0, 1, 0), # Direction the sensor is pointing
    50,                   # Number of horizontal rays
    chrono.CH_C_DEG_TO_RAD * 20.0, # FOV,
    50.0                 # Maximum distance
)
lidar.SetName("Lidar Sensor")
# Set visualization parameters
lidar.SetShowRays(True)
lidar.SetRaysVisualizationLength(20)
lidar.SetPointSize(0.01)
# Set data filtering parameters
lidar.SetNoiseModel(sens.ChLidarSensor.NoiseModel.GAUSSIAN)
lidar.SetGaussianNoise(0.005)
# Set data saving parameters
lidar.SetSaveHorizontalRays(True)
lidar.SetSaveVerticalRays(False)
# Add the sensor to the manager
manager.AddSensor(lidar)

# ---------------------------------------------------------------------
#  Simulation loop
# ---------------------------------------------------------------------
# Time to simulate for
t_end = 10.0
time = 0

while time < t_end:
    # Update the position of the lidar in orbit
    orbit_radius = 5.0
    orbit_speed = 0.5
    angle = orbit_speed * time
    lidar.SetOffsetPos(chrono.ChVectorD(orbit_radius * math.cos(angle), 0, orbit_radius * math.sin(angle)))

    # Advance simulation by one step
    manager.Update()
    system.DoStepDynamics(time_step)

    # Print lidar buffer data
    print(f"Time: {time:.2f}")
    print(f"Lidar data buffer size: {lidar.GetPointCloud().size()}")

    time += time_step

# ---------------------------------------------------------------------
#  Save sensor data to file
# ---------------------------------------------------------------------
manager.ExportData("./sensor_output/test_output")