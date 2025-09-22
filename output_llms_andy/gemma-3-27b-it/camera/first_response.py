import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# ---
#  Create the simulation system and add items
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Load the triangular mesh from a Wavefront .obj file
mesh_file = "cube.obj"  # Replace with your mesh file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadFromFile(mesh_file)

# Create a ChBody from the mesh
body = chrono.ChBodyEasy()
body.AddAsset(mesh)
body.SetBodyFixed(True)  # Make the body fixed
system.Add(body)

# Set initial position of the body
body.SetPos(chrono.ChVectorD(0, 0, 0))

# ---
# Create a camera sensor and sensor manager
# ---

# Create a sensor manager
sensor_manager = chrono.ChSensorManager()
sensor_manager.AddSensor(body)

# Create a camera sensor
camera = chrono.ChCameraSensor()
camera.SetBody(body)
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetNearClip(0.1)
camera.SetFarClip(100)
camera.SetNoise(0.01, 0.01)  # Add some noise
sensor_manager.AddSensor(camera)

# Create a visualization for the camera images
camera_vis = chronoirr.ChVisualisationCamera()
camera_vis.SetSensor(camera)
camera_vis.SetRenderMode(chronoirr.ChVisualisationCamera.RENDER_MODE_DEPTH)  # Show depth map
sensor_manager.AddVisualizer(camera_vis)


# ---
#  Create an Irrlicht application to visualize the system
# ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -3))
vis.AddTypicalLights()


# ---
#  Simulation loop
# ---

# Time step
time_step = 0.01

# Simulation duration
simulation_duration = 10

# Orbit parameters
orbit_radius = 2.0
orbit_speed = 0.5

current_time = 0.0
while vis.Run() and current_time < simulation_duration:
    # Update camera position in orbit
    angle = orbit_speed * current_time
    camera_x = orbit_radius * np.cos(angle)
    camera_z = orbit_radius * np.sin(angle)
    vis.GetCamera().SetPos(chrono.ChVectorD(camera_x, 1.5, camera_z))
    vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))

    # Advance the simulation
    system.DoStepDynamics(time_step)

    # Get camera buffer data
    if camera.IsDataAvailable():
        buffer_data = camera.GetBufferData()
        print(f"Camera buffer data at time {current_time:.2f}: {buffer_data}")

    current_time += time_step