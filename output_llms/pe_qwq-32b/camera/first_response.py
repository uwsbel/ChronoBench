import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as chsensor
import math
import numpy as np

# Initialize system and visualization
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mesh Camera Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChVectorD(0, 3, 10))  # Initial camera view
vis.AddTypicalLights()

# Load triangular mesh from .obj file (replace with actual path)
mesh_file = 'my_mesh.obj'  # Update this path
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetYoungModulus(2e7)
material.SetRestitution(0.1)

mesh_body = chrono.ChBodyEasyMesh(
    chrono.GetChronoDataFile(mesh_file),
    1000,  # Density
    True,  # Visualize
    False,  # No collision
    material
)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetFixed(True)
system.Add(mesh_body)

# Initialize sensor manager and camera sensor
sensor_mgr = chsensor.ChSensorManager(system)
cam_sensor = chsensor.ChCameraSensor()
cam_sensor.SetName("orbit_cam")
cam_sensor.SetResolution(800, 600)
cam_sensor.SetFov(60.0)  # Field of view in degrees
cam_sensor.SetPixelFormat(chsensor.PixelType_RGB)  # Ensure RGB output
cam_sensor.SetPosition(chrono.ChVectorD(0, 0, 5))  # Initial position above mesh
cam_sensor.SetRotation(chrono.Q_from_Euler123(chrono.ChVectorD(0, 0, 0)))  # Initial orientation

# Add noise filters (Gaussian noise example)
cam_sensor.AddNoiseFilter(chsensor.ChNoiseFilterGaussian(0.0, 0.1))

sensor_mgr.AddSensor(cam_sensor)
sensor_mgr.SetVisualizationType(chsensor.VisualizationType_RGB)  # Enable sensor visualization

# Simulation parameters
time_step = 0.01
angular_velocity = 0.1  # rad/s orbit speed
radius = 5.0  # Distance from mesh center

# Simulation loop
while vis.Run():
    current_time = system.GetChTime()
    angle = angular_velocity * current_time
    
    # Update camera position in orbit
    x = radius * math.cos(angle)
    z = radius * math.sin(angle)
    cam_pos = chrono.ChVectorD(x, 0, z)
    cam_sensor.SetPosition(cam_pos)
    
    # Set camera orientation to look at mesh center (0,0,0)
    look_dir = chrono.ChVectorD(0, 0, 0) - cam_pos
    up_dir = chrono.ChVectorD(0, 1, 0)
    cam_sensor.SetRotation(chrono.Q_from_dir_dir(up_dir, look_dir))
    
    # Update sensor manager to capture image
    sensor_mgr.Update()
    
    # Render and step simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    # Access and print camera buffer data
    buffer = cam_sensor.GetSensor().GetColorBuffer()
    if buffer is not None:
        print(f"Frame {int(current_time/time_step)}:")
        print(f"  Resolution: {buffer.shape}")
        print(f"  Min pixel value: {buffer.min()}, Max pixel value: {buffer.max()}")

# Cleanup (if needed)
vis.Close()