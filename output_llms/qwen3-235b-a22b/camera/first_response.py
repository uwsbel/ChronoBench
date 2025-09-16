import pychrono as chrono
import pychrono.sensor as sens
import math

# Initialize the PyChrono system
system = chrono.ChSystemNSC()

# Load the triangular mesh from .obj file
mesh = chrono.ChTriangleMeshConnected()
# Replace 'path/to/mesh.obj' with the actual mesh file path
mesh.LoadWavefrontMesh('path/to/mesh.obj', True, True)

# Create a fixed body and attach the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.GetCollisionModel().ClearModel()
body.GetCollisionModel().AddTriangleMesh(mesh, False, False, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
body.GetCollisionModel().BuildModel()
system.Add(body)

# Add visualization asset for the mesh
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh)
body.AddVisualShape(vis_shape)

# Create Irrlicht visualization
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Mesh Visualization')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))

# Create sensor manager
manager = sens.ChSensorManager(system)

# Create camera sensor
camera = sens.ChCameraSensor(
    body,                    # parent body
    60,                    # horizontal field of view (degrees)
    640,                   # image width
    480,                   # image height
    chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.QUNIT)  # initial offset pose
)
camera.SetName("Camera")
camera.SetLag(0)  # No lag
camera.SetCollectionWindow(0.01)  # Time to collect data per frame

# Add filters to the camera
camera.PushFilter(sens.ChFilterNoiseGaussian(0.0, 0.02))  # Gaussian noise
camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Feed"))  # Visualization

# Add camera to manager
manager.AddSensor(camera)

# Simulation loop
orbit_radius = 2.0
omega = 0.5  # Angular velocity (rad/s)
step_size = 1e-3
end_time = 10.0

while vis.Run():
    current_time = system.GetChTime()
    
    # Calculate new camera position and orientation
    theta = omega * current_time
    cam_pos = chrono.ChVectorD(orbit_radius * math.cos(theta), 0, orbit_radius * math.sin(theta))
    
    # Compute rotation to look at the origin
    look_dir = chrono.ChVectorD(0, 0, 0) - cam_pos
    rot = chrono.Q_from_2vectors(chrono.ChVectorD(0, 0, -1), look_dir)
    
    # Update camera's position and orientation
    camera.SetOffsetPose(chrono.ChFrameD(cam_pos, rot))
    
    # Step the simulation
    system.DoStepDynamics(step_size)
    
    # Update sensors
    manager.Update()
    
    # Print camera buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer is not None:
        print(f"Time: {current_time:.3f}, Buffer data sample: {buffer[0, 0]}")