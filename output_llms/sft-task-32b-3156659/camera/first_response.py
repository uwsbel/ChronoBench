import pychrono as chrono
import pychrono.sensor as sens
import math

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No gravity

# Create a fixed body with a triangular mesh
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("path/to/your/mesh.obj", False, False)  # Replace with actual .obj path
mesh_asset = chrono.ChVisualShapeTriangleMesh()
mesh_asset.SetMesh(mesh)
mesh_asset.SetMutable(False)

mesh_body = chrono.ChBodyEasySphere(1, 1000, True, True)
mesh_body.SetFixed(True)
mesh_body.SetName("Fixed Mesh Body")
mesh_body.GetVisualShape(0).Remove()
mesh_body.AddVisualShape(mesh_asset)
system.Add(mesh_body)

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)

# Create camera sensor
camera = sens.ChCameraSensor(
    mesh_body,  # Attach to fixed mesh body
    30,         # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVector3d(2, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),
    1280, 720,  # Resolution
    math.radians(60)  # FOV
)
camera.SetName("Orbit Camera")
camera.SetLensModel(sens.ChOpticalModelType.Pinhole)

# Add noise and visualization filters
camera.PushFilter(sens.ChFilterCameraNoiseGaussian(0.0, 0.05))
camera.PushFilter(sens.ChFilterVisualize(1280, 720))

sensor_manager.AddSensor(camera)

# Simulation parameters
time_step = 1.0 / 30.0
orbit_radius = 2.0
orbit_height = 2.0
angle = 0.0

# Simulation loop
while system.GetChTime() < 10.0:
    # Update camera orbit position
    angle += 0.05
    cam_x = orbit_radius * math.cos(angle)
    cam_z = orbit_height + 0.5 * math.sin(angle)  # Slight vertical oscillation
    camera.SetOffsetPos(chrono.ChVector3d(cam_x, 0, cam_z))
    camera.SetOffsetRot(chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))

    system.DoStepDynamics(time_step)
    sensor_manager.Update()

    # Print camera buffer data
    if camera.HasLatestData():
        img = camera.GetLatestImage()
        print(f"Camera buffer: {img.width}x{img.height}, Timestamp: {img.timestamp}")

    # Small delay for visualization
    chrono.ChRealtimeStepDamping(time_step)