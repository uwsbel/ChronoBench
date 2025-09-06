import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensor

# Set the path to the Chrono data directory.  Adjust this if your data is elsewhere.
chrono.SetChronoDataPath("./chrono_data")

# ---------------------------------------------------------------------
#
#  CREATE THE SIMULATION
#  ---------------------------------------------------------------------

# Create a physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
#
#  CREATE THE MESH BODY
#  ---------------------------------------------------------------------

# Load the mesh from a file (replace with your .obj file)
mesh_file = "sphere.obj"  # Replace with your mesh file
mesh_path = os.path.join(chrono.GetChronoDataFile(""), mesh_file)

# Create the body and add the mesh shape
body = chrono.ChBody()
body.SetBodyFixed(True)  # Make the mesh fixed
body.SetCollide(True)
body.SetMass(1.0)
body.SetPos(chrono.ChVector3d(0, 0, 0))

mesh_shape = chrono.ChVisualShapeFile()
mesh_shape.SetFile(mesh_path)
body.AddVisualShape(mesh_shape)

# Add the body to the system
sys.Add(body)

# ---------------------------------------------------------------------
#
#  CREATE THE CAMERA SENSOR
#  ---------------------------------------------------------------------

# Create a sensor manager
sensor_manager = sensor.ChSensorManager(sys)

# Create a camera sensor
camera = sensor.ChCameraSensor()
camera.SetBody(body)
camera.SetOffset(chrono.ChVector3d(0, 0, 2))  # Offset from the body's center
camera.SetResolution(640, 480)
camera.SetFov(math.pi / 3.0)
camera.SetNear(0.1)
camera.SetFar(100.0)

# Add noise filter (example: Gaussian noise)
noise_filter = sensor.ChNoiseGaussian()
noise_filter.SetMean(0.0)
noise_filter.SetStddev(0.01)
camera.AddNoiseFilter(noise_filter)

# Add visualization (example: grayscale)
visualization = sensor.ChSensorVisualizationGrayscale()
camera.AddVisualization(visualization)

# Register the camera with the sensor manager
sensor_manager.AddSensor(camera)

# ---------------------------------------------------------------------
#
#  VISUALIZATION
#  ---------------------------------------------------------------------

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()
vis.AddSkyBox()

# ---------------------------------------------------------------------
#
#  SIMULATION LOOP
#  ---------------------------------------------------------------------

time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation
    sys.DoStepDynamics(time_step)

    # Get the camera buffer data
    buffer = camera.GetBuffer()
    if buffer is not None:
        print(f"Camera buffer size: {len(buffer)}")
        # Process the buffer data (e.g., save to file, display)
        # For demonstration, just print the first few values
        if len(buffer) > 10:
            print(f"First 10 buffer values: {buffer[:10]}")
    else:
        print("Camera buffer is empty.")