import pychrono as chrono
import pychrono.sensor as sens
import math

# Initialize the PyChrono system
system = chrono.ChSystemNSC()

# Load the triangular mesh from a Wavefront .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('mesh.obj', True, True)  # Replace with actual path
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)

# Create a dynamic body for the camera
camera_body = chrono.ChBody()
camera_body.SetPos(chrono.ChVectorD(2, 0, 1))  # Initial position
system.Add(camera_body)

# Create the sensor manager
manager = sens.ChSensorManager(system)

# Create the camera sensor
camera = sens.ChCameraSensor(
    camera_body,
    30,  # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    1280,  # Width
    720,   # Height
    chrono.CH_C_PI / 3  # Field of view
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0)
manager.AddSensor(camera)

# Add Gaussian noise filter
noise_filter = sens.ChFilterNoiseModelGaussian(camera, 0.1)
manager.AddFilter(noise_filter)

# Add visualization filter
vis_filter = sens.ChFilterVisualize(camera, 1280, 720, "Camera Feed")
manager.AddFilter(vis_filter)

# Simulation parameters
time_step = 0.01
end_time = 10.0
orbit_radius = 2.0
omega = 0.5  # Angular velocity (rad/s)
height = 1.0

# Simulation loop
while system.GetChTime() < end_time:
    # Calculate new camera position and orientation
    current_time = system.GetChTime()
    angle = omega * current_time
    new_x = orbit_radius * math.cos(angle)
    new_y = orbit_radius * math.sin(angle)
    new_z = height
    new_pos = chrono.ChVectorD(new_x, new_y, new_z)
    camera_body.SetPos(new_pos)
    
    # Calculate rotation to face the origin
    dir_to_origin = chrono.ChVectorD(0, 0, 0) - new_pos
    dir_to_origin.Normalize()
    rot_quat = chrono.ChQuaternionD()
    rot_quat.SetFromTwoVectors(chrono.ChVectorD(0, 0, -1), dir_to_origin)
    camera_body.SetRot(rot_quat)
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    
    # Update sensors
    manager.Update()
    
    # Print camera buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer:
        print(f"Time: {current_time:.2f}s, Buffer timestamp: {buffer.GetTimestamp():.2f}, Data available: {buffer.GetData() is not None}")