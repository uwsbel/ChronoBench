import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '../data/'))

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity for this simulation

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Orbiting Camera Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChVectorD(0, 0, 1))

# 2. Add the required physical systems and objects
# Load the mesh from OBJ file (replace with your actual file path)
mesh_filename = "path/to/your/mesh.obj"  # Update this path
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_filename, False, True)

# Create a fixed body with the mesh
body = chrono.ChBody()
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetBodyFixed(True)
body.GetVisualModel().AddTriangleMesh(mesh)
system.Add(body)

# 3. Set up the camera sensor and manager
sensor_manager = chrono.ChSensorManager(system)
sensor_manager.SetSensorsUpdateMode(chrono.SensorUpdateMode_ASYNCHRONOUS)

# Create a camera sensor
camera = chrono.ChCameraSensor(
    body,                          # parent body
    10,                            # update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2)),  # initial position
    640,                           # horizontal resolution
    480,                           # vertical resolution
    chrono.ChCameraSensor::LENS_PERSPECTIVE,
    1.414,                         # aspect ratio
    0.05,                          # focal length
    chrono.ChCameraSensor::FILTER_BILINEAR
)

# Add noise filters to the camera
camera.SetNoiseFilter(chrono.ChCameraSensor::NoiseModel_GAUSSIAN, 0.05)  # 5% noise
camera.SetBlurFilter(chrono.ChCameraSensor::BlurFilter_GAUSSIAN, 3)  # 3x3 Gaussian blur

# Add the camera to the sensor manager
sensor_manager.AddSensor(camera)

# Initialize the camera visualization
camera_vis = chrono.ChCameraSensorVisualization(camera)
camera_vis.SetWindowTitle("Camera View")
camera_vis.SetWindowSize(640, 480)
camera_vis.Initialize()

# Simulation parameters
time_step = 0.01
sim_time = 10.0
orbit_radius = 2.0
orbit_speed = 0.5

# Simulation loop
current_time = 0
while vis.Run() and current_time < sim_time:
    # Update the camera position in an orbit
    angle = orbit_speed * current_time
    camera_pos = chrono.ChVectorD(
        orbit_radius * np.cos(angle),
        orbit_radius * np.sin(angle),
        1.0
    )
    camera.SetCameraTransform(chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 0, 1))))

    # Advance the simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the sensor manager
    sensor_manager.Update()

    # Print camera buffer data
    if camera.HasData():
        print(f"Time: {current_time:.2f}s - Camera buffer size: {camera.GetBufferSize()}")

    # Step the simulation
    system.DoStepDynamics(time_step)
    current_time += time_step

# Clean up
camera_vis.Shutdown()
vis.Shutdown()