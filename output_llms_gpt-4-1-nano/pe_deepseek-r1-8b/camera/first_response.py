import os
import math
import numpy as np
import chrono as chrono
from chronoirr import ChVisualSystemIrrlicht
from pychrono import ChSensorManager

# Initialize the system
sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('path/to/data')  # Replace with your data path

# Initialize visualization
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddSkyBox()

# Load the mesh
mesh_shape = chrono.ChVisualShape()
mesh_shape.LoadFromWavefront('triangular_mesh.obj')
mesh_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color
sys.AddVisualShape(mesh_shape)

# Set up camera
camera_pos = chrono.ChVector3d(5, 5, 5)
camera_rot = chrono.Quat()
camera_rot.SetFromAxisAng(chrono.CH_PI/2, 1, 0, 0)  # Look towards origin

# Add camera to the system
sys.AddCamera(camera_pos, camera_rot)

# Create motor joint for camera movement
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sys.GetBody(0), sys.GetBody(1), chrono.ChCoordsysD(camera_pos, camera_rot))

# Set up the motor's motion
motor.SetSpeedFunction(chrono.ChFunction_Sine(0.05, 1.0, True))  # Y-axis movement

# Initialize sensor manager
sensor_manager = ChSensorManager(sys)
sensor_manager.AddSensor(0, 'camera', 0)  # Add camera sensor
sensor_manager.Enable(chrono.ChSensorMode.Off)  # Disable automatic capture

# Set noise filter parameters
noise_filter = chrono.ChSensorNoiseFilter()
noise_filter.SetNoiseLevel(0.1)
noise_filter.SetFilterSize(5)
sensor_manager.SetNoiseFilter(noise_filter)

# Enable image capture
sensor_manager.Enable(chrono.ChSensorMode.On)

# Print the camera data at each step
def print_camera_data():
    pos = sys.GetBody(0).GetPos()
    rot = sys.GetBody(0).GetRot()
    img = sensor_manager.GetImage()
    
    print(f"Camera Position: {pos}")
    print(f"Camera Rotation: {rot}")
    print(f"Image Data: {img}")
    print("----------------------------")

# Run the simulation
while True:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    motor.Update()
    sys.DoStepDynamics(0.01)
    print_camera_data()

import os
import math
import numpy as np
import chrono as chrono
from chronoirr import ChVisualSystemIrrlicht
from pychrono import ChSensorManager

# Set up the PyChrono environment
sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('path/to/data')  # Replace with your data path

# Initialize the visualization system
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddSkyBox()

# Load and visualize the triangular mesh
mesh_shape = chrono.ChVisualShape()
mesh_shape.LoadFromWavefront('triangular_mesh.obj')
mesh_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color
sys.AddVisualShape(mesh_shape)

# Set up the camera position and rotation
camera_pos = chrono.ChVector3d(5, 5, 5)
camera_rot = chrono.Quat()
camera_rot.SetFromAxisAng(chrono.CH_PI/2, 1, 0, 0)  # Look towards origin

# Add camera to the system
sys.AddCamera(camera_pos, camera_rot)

# Create a motor joint for camera movement
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sys.GetBody(0), sys.GetBody(1), chrono.ChCoordsysD(camera_pos, camera_rot))

# Set the motor's motion function
motor.SetSpeedFunction(chrono.ChFunction_Sine(0.05, 1.0, True))  # Y-axis movement

# Set up the sensor manager
sensor_manager = ChSensorManager(sys)
sensor_manager.AddSensor(0, 'camera', 0)  # Add camera sensor
sensor_manager.Enable(chrono.ChSensorMode.Off)  # Disable automatic capture

# Add noise filter to the sensor
noise_filter = chrono.ChSensorNoiseFilter()
noise_filter.SetNoiseLevel(0.1)
noise_filter.SetFilterSize(5)
sensor_manager.SetNoiseFilter(noise_filter)

# Enable image capture
sensor_manager.Enable(chrono.ChSensorMode.On)

# Function to print camera data
def print_camera_data():
    pos = sys.GetBody(0).GetPos()
    rot = sys.GetBody(0).GetRot()
    img = sensor_manager.GetImage()
    
    print(f"Camera Position: {pos}")
    print(f"Camera Rotation: {rot}")
    print(f"Image Data: {img}")
    print("----------------------------")

# Run the simulation loop
while True:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    motor.Update()
    sys.DoStepDynamics(0.01)
    print_camera_data()