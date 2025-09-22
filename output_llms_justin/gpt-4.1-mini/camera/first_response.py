import pychrono as chrono
import pychrono.sensor as sens
import os

# 1. Initialize the PyChrono environment and core components.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Make sure path is set properly if needed

sys = chrono.ChSystemNSC()

# 2. Add the required physical systems and objects as specified.

# Create a fixed body with a triangular mesh loaded from a Wavefront .obj file.
body = chrono.ChBody()
body.SetBodyFixed(True)
sys.Add(body)

# Load the mesh from a .obj file (use a sample mesh from Chrono data or specify your own)
mesh_path = chrono.GetChronoDataPath() + "sensor/mesh.obj"
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, True, True)

# Create a visual shape from the mesh and attach to body
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_shape.SetMutable(False)
body.AddVisualShape(mesh_shape)

# 3. Set necessary default parameters such as positions, forces, and interactions.

# Initialize the sensor manager and attach it to the system and its timestep
sensor_manager = sens.ChSensorManager(sys)
sensor_manager.SetVerbose(True)
sensor_manager.SetWindowSize(800, 600)
sensor_manager.SetProcessingThreads(4)

# Create a camera sensor attached to the body
cam = sens.ChCameraSensor(body)
cam.SetName("Camera Sensor")

# Set timestep between images capture (50 Hz)
cam.SetUpdateRate(50)
cam.SetResolution(800, 600)
cam.SetFOV(1.0)  # approx ~57deg FOV

# Position the camera at some initial offset relative to the body
cam_pose = chrono.ChFrameD()
cam_pose.SetPos(chrono.ChVectorD(1, 0, 0.5))  # Initial offset
cam.SetOffsetPose(cam_pose)

# Add noise and filters to the camera sensor
cam.AddFilter(sens.ChFilterRGBDNoise(0.005))  # very small noise
cam.AddFilter(sens.ChFilterVisualize(800, 600, "Camera View"))
cam.AddFilter(sens.ChFilterSave("camera_image.png"))  # Saves to PNG each update (optional)

sensor_manager.AddSensor(cam)

# Simulation parameters
timestep = 0.01
sim_time = 5  # seconds

# Orbit parameters for camera around the fixed body:
import math

radius = 1.5
height = 0.5
angular_speed = 0.4 * 2 * math.pi  # radians per second (0.4 revolutions per second)

steps = int(sim_time / timestep)

for step in range(steps):
    time = step * timestep

    # Orbit the camera about body (around vertical axis Z)
    angle = angular_speed * time
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z = height

    cam.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(x, y, z), chrono.Q_from_AngAxis(-angle, chrono.ChVectorD(0, 0, 1))))

    # Advance simulation
    sys.DoStepDynamics(timestep)
    sensor_manager.Update()

    # Access camera buffer data (RGB)
    rgb_buffer = cam.GetMostRecentRGB()
    # Print out some info about the buffer (dimensions and a pixel sample)
    width = cam.GetWidth()
    height = cam.GetHeight()

    # Just print first pixel RGB value as sample
    if rgb_buffer is not None and len(rgb_buffer) >= 3:
        pix_r = rgb_buffer[0]
        pix_g = rgb_buffer[1]
        pix_b = rgb_buffer[2]
        print(f"Time: {time:.2f}s, Camera RGB first pixel: R={pix_r} G={pix_g} B={pix_b}")
    else:
        print(f"Time: {time:.2f}s, No RGB data available")

print("Simulation finished.")