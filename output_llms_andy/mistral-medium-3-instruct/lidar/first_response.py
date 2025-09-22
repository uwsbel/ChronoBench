import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Replace with your actual path

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity for this simulation

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Simulation with OBJ Mesh")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChVectorD(0, 0, 0))

# Create a fixed body with the OBJ mesh
mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# Load the OBJ mesh (replace with your actual OBJ file path)
mesh_path = "path/to/your/mesh.obj"  # Replace with your actual path
if not os.path.exists(mesh_path):
    raise FileNotFoundError(f"Mesh file not found: {mesh_path}")

mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, True, True)

mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetName("OBJ_Mesh")
mesh_body.AddAsset(mesh_asset)

# Create a lidar sensor
sensor_manager = sensors.ChSensorManager(system)
lidar = sensors.ChLidarSensor()

# Configure the lidar
lidar.SetName("Lidar_Sensor")
lidar.SetUpdateRate(30)  # Hz
lidar.SetScanRate(100)   # Scans per second
lidar.SetVerticalResolution(32)  # Number of vertical channels
lidar.SetVerticalFOV(30)  # Degrees
lidar.SetHorizontalFOV(360)  # Degrees
lidar.SetMinRange(0.1)    # Minimum detection range (m)
lidar.SetMaxRange(50)     # Maximum detection range (m)

# Add noise to the lidar measurements
noise_model = sensors.ChNoiseGaussian(0.01)  # 1% noise
lidar.AddNoiseModel(noise_model)

# Create a visualization for the lidar
lidar_vis = sensors.ChLidarVisualization(system, lidar)
sensor_manager.AddSensor(lidar)

# Create a body to hold the lidar
lidar_body = chrono.ChBody()
system.Add(lidar_body)
lidar_body.SetPos(chrono.ChVectorD(0, 0, 0))
lidar_body.SetBodyFixed(False)  # We'll move it manually

# Attach the lidar to the body
lidar.SetBody(lidar_body)

# Set up data saving
lidar_data = sensors.ChLidarData()
lidar.SetData(lidar_data)
lidar_data.SetSaveToFile(True)
lidar_data.SetFileName("lidar_data.txt")

# Simulation parameters
time_step = 0.01
end_time = 10.0
current_time = 0

# Simulation loop
while vis.Run() and current_time < end_time:
    # Update the lidar position in an orbit around the mesh
    angle = current_time * 0.5  # Orbit speed
    radius = 2.0  # Orbit radius
    lidar_body.SetPos(chrono.ChVectorD(
        radius * np.cos(angle),
        radius * np.sin(angle),
        1.0  # Fixed height above the mesh
    ))

    # Point the lidar toward the center
    lidar_body.SetRot(chrono.Q_from_AngAxis(angle + np.pi, chrono.ChVectorD(0, 0, 1)))

    # Advance the simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the sensor manager
    sensor_manager.Update()

    # Print the lidar buffer data
    if lidar_data.GetNumPoints() > 0:
        print(f"Time: {current_time:.2f}s - Points detected: {lidar_data.GetNumPoints()}")
        # Print first few points as example
        for i in range(min(5, lidar_data.GetNumPoints())):
            point = lidar_data.GetPoint(i)
            print(f"  Point {i}: Distance={point.distance:.2f}m, Intensity={point.intensity:.2f}")

    # Advance time
    system.DoStepDynamics(time_step)
    current_time += time_step

# Save the final lidar data
lidar_data.SaveToFile()