import pychrono as chrono
import pychrono.sensor as sens
import numpy as np

# Initialize PyChrono environment
chrono.SetChronoDataPath('/path/to/chrono/data')

# Create a PyChrono system
sys = chrono.ChSystemNSC()

# Load a triangular mesh from a Wavefront .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/tractor.obj'))

# Create a body with the loaded mesh and add it to the system
body_mesh = chrono.ChBody()
body_mesh.SetBodyFixed(True)  # Fix the body in space
body_mesh_shape = chrono.ChVisualShapeTriangleMesh()
body_mesh_shape.SetMesh(mesh)
body_mesh.AddVisualShape(body_mesh_shape)
sys.Add(body_mesh)

# Create a new body to hold the camera
body_camera = chrono.ChBody()
body_camera.SetPos(chrono.ChVector3d(0, 2, 0))  # Initial camera position
sys.Add(body_camera)

# Create a camera sensor
camera = sens.ChCameraSensor(
    body_camera,  # Body camera is attached to
    10,  # Update rate in Hz
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI, chrono.ChVector3d(0, 1, 0))),  # Offset pose
    640,  # Image width
    480,  # Image height
    chrono.CH_PI / 4  # FOV
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8RGBA8())  # Convert to RGBA8 format
camera.PushFilter(sens.ChFilterGrayscale())  # Convert to grayscale
camera.PushFilter(sens.ChFilterNoisePixDep(0.03, 0.001, 123))  # Add noise
camera.PushFilter(sens.ChFilterVisualize(640, 480, 1))  # Visualize camera output

# Add the camera to the sensor manager
manager = sens.ChSensorManager(sys)
manager.AddSensor(camera)

# Simulation loop
time_step = 0.02  # Time step for the simulation
t = 0  # Current time
while t < 10:  # Simulate for 10 seconds
    # Update the camera body's position to orbit around the mesh
    angle = t * 0.1  # Adjust the speed of the orbit
    radius = 2
    x = radius * np.cos(angle)
    z = radius * np.sin(angle)
    body_camera.SetPos(chrono.ChVector3d(x, 2, z))
    
    # Update the simulation
    sys.DoStepDynamics(time_step)
    manager.Update()
    
    # Access and print camera buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer.HasData():
        print(f"Camera Buffer at time {t}: {buffer.GetRGBA8().shape}")
    
    # Increment time
    t += time_step

print("Simulation completed.")