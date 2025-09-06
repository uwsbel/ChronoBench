import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os

# 1. Initialize PyChrono environment
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '..', 'data'))
chrono.SetChronoOutputPath(os.path.join(os.path.dirname(__file__), '..', 'output'))

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity needed for fixed body

# 2. Add visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Triangular Mesh with Orbiting Camera')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.SetCameraMoveScale(0.05)

# 3. Load the triangular mesh (replace with your OBJ file path)
mesh_path = "path/to/your/mesh.obj"  # Update this path
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, False, True)

# Create a fixed body with the mesh
fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
fixed_body.GetVisualModel().AddTriangleMesh(mesh)
fixed_body.GetCollisionModel().AddTriangleMesh(mesh)
system.Add(fixed_body)

# 4. Create camera sensor
camera = sensors.ChCameraSensor(
    fixed_body,                # Parent body
    10,                        # Update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(5, 0, 0), chrono.QUNIT),  # Initial position
    640,                       # Width
    480,                       # Height
    1.0                        # Focal length
)

# Configure camera parameters
camera.SetLensModel(sensors.ChLensModelPINHOLE)
camera.SetImagePlaneSize(640, 480)
camera.SetClippingPlanes(0.1, 100)
camera.SetUseColorTexture(True)

# Add noise filters
noise_filter = sensors.ChFilterGaussianNoise()
noise_filter.SetNoiseLevel(0.01)  # 1% noise
camera.AddFilter(noise_filter)

# Add visualization filter
visualization_filter = sensors.ChFilterVisualize(640, 480, "Camera View")
camera.AddFilter(visualization_filter)

# Create sensor manager
sensor_manager = sensors.ChSensorManager(system)
sensor_manager.AddSensor(camera)

# 5. Simulation parameters
time_step = 0.01
end_time = 10.0

# 6. Simulation loop
while vis.Run():
    # Update camera position in orbit
    time = system.GetChTime()
    radius = 5.0
    angle = time * 0.5  # Adjust angular speed as needed

    # Circular orbit around the mesh
    cam_pos = chrono.ChVectorD(
        radius * np.cos(angle),
        radius * np.sin(angle),
        2.0  # Keep some height
    )

    # Point camera at origin
    camera.SetCameraToBodyOffset(chrono.ChFrameD(cam_pos, chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.ChVectorD(0,0,1))))

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Print camera buffer info
    if camera.GetLatestFrame() is not None:
        frame = camera.GetLatestFrame()
        print(f"Time: {time:.2f}s | Camera position: {cam_pos} | Buffer size: {frame.GetBufferWidth()}x{frame.GetBufferHeight()}")

    # Step simulation
    system.DoStepDynamics(time_step)

    # Check if simulation time has ended
    if time >= end_time:
        break

# Clean up
sensor_manager.RemoveSensor(camera)