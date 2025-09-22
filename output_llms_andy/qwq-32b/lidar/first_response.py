import pychrono as chrono
import pychrono.sensor as csensor
import math

# Simulation parameters
time_step = 0.01
end_time = 10.0
angular_speed = 0.1  # radians per second
radius = 5.0  # orbit radius

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # gravity

# Create Irrlicht visualizer
vis = chrono.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Lidar Mesh Simulation')
vis.SetSymbolscale(0.002)
vis.SetShadowFBO(True)
system.SetVisualSystem(vis)

# Add a static body with the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)
body.SetPos(chrono.ChVectorD(0, 0, 0))

# Load the mesh (replace with actual file path)
mesh = chrono.ChTriangleMeshShape()
mesh.SetName("mesh")
mesh.SetMesh(chrono.import_obj("my_mesh.obj"))  # Replace with actual .obj file path
body.AddAsset(mesh)
system.Add(body)

# Create sensor manager
sensor_mgr = csensor.ChSensorManager(system)
sensor_mgr.SetRenderTarget(csensor.ChSensorManager.RT_OFFSCREEN)

# Lidar parameters
lidar_name = 'my_lidar'
lidar_hfov = 90.0  # degrees
lidar_vfov = 30.0
lidar_near = 0.1
lidar_far = 100.0
lidar_width = 512
lidar_height = 480

# Initial position (before orbit starts)
initial_angle = 0.0
initial_x = radius * math.cos(initial_angle)
initial_y = radius * math.sin(initial_angle)
initial_pos = chrono.ChVectorD(initial_x, initial_y, 0.0)

# Direction towards origin
initial_dir = chrono.ChVectorD(-initial_x, -initial_y, 0.0).Get_normalized()
initial_up = chrono.ChVectorD(0, 0, 1)
initial_rot = chrono.Q_from_A_frame(initial_dir, initial_up)
initial_frame = chrono.ChFrameD(initial_pos, initial_rot)

# Create the lidar sensor
lidar = sensor_mgr.AddLidar(lidar_name, initial_frame, lidar_hfov, lidar_vfov,
                           lidar_near, lidar_far, lidar_width, lidar_height)

# Configure noise filter (Gaussian noise with 5% standard deviation)
noise = csensor.ChNoiseGaussian(0.05)
lidar.SetNoise(noise)

# Enable saving frames to files
lidar.SetSaveFrames(True, 'lidar_frames/frame_')

# Initialize the sensor manager
sensor_mgr.Init()

# Visualization settings
sensor_mgr.SetVizType(chrono.render.RendererType_IRR)

# Setup visualization
system.SetupVisualization()
vis.SetCameraPosition(chrono.ChVectorD(10, 10, 10))
vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))

# Simulation loop
while system.GetChTime() < end_time:
    current_time = system.GetChTime()
    
    # Calculate current angle and position for the orbit
    angle = angular_speed * current_time
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    pos = chrono.ChVectorD(x, y, 0.0)
    
    # Compute direction towards origin
    dir = chrono.ChVectorD(-x, -y, 0.0).Get_normalized()
    up = chrono.ChVectorD(0, 0, 1)
    rot = chrono.Q_from_A_frame(dir, up)
    frame = chrono.ChFrameD(pos, rot)
    
    # Update sensor position and orientation
    sensor_mgr.UpdateSensor(lidar, frame, current_time)
    
    # Retrieve and print lidar buffer data
    buffer = lidar.GetBuffer()
    print(f"Time {current_time:.2f}: Lidar points = {len(buffer)}")
    if buffer:
        print(f"First point distance: {buffer[0].distance:.3f} m")
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Cleanup
chrono.GetLog() << "Simulation completed.\n"

import pychrono as chrono
import pychrono.sensor as csensor
import math

# Simulation parameters
time_step = 0.01
end_time = 10.0
angular_speed = 0.1  # radians per second
radius = 5.0  # orbit radius

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # gravity

# Create Irrlicht visualizer
vis = chrono.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Lidar Mesh Simulation')
vis.SetSymbolscale(0.002)
vis.SetShadowFBO(True)
system.SetVisualSystem(vis)

# Add a static body with the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)
body.SetPos(chrono.ChVectorD(0, 0, 0))

# Load the mesh (replace with actual file path)
mesh = chrono.ChTriangleMeshShape()
mesh.SetName("mesh")
mesh.SetMesh(chrono.import_obj("my_mesh.obj"))  # Replace with actual .obj file path
body.AddAsset(mesh)
system.Add(body)

# Create sensor manager
sensor_mgr = csensor.ChSensorManager(system)
sensor_mgr.SetRenderTarget(csensor.ChSensorManager.RT_OFFSCREEN)

# Lidar parameters
lidar_name = 'my_lidar'
lidar_hfov = 90.0  # degrees
lidar_vfov = 30.0
lidar_near = 0.1
lidar_far = 100.0
lidar_width = 512
lidar_height = 480

# Initial position (before orbit starts)
initial_angle = 0.0
initial_x = radius * math.cos(initial_angle)
initial_y = radius * math.sin(initial_angle)
initial_pos = chrono.ChVectorD(initial_x, initial_y, 0.0)

# Direction towards origin
initial_dir = chrono.ChVectorD(-initial_x, -initial_y, 0.0).Get_normalized()
initial_up = chrono.ChVectorD(0, 0, 1)
initial_rot = chrono.Q_from_A_frame(initial_dir, initial_up)
initial_frame = chrono.ChFrameD(initial_pos, initial_rot)

# Create the lidar sensor
lidar = sensor_mgr.AddLidar(
    lidar_name,
    initial_frame,
    lidar_hfov,
    lidar_vfov,
    lidar_near,
    lidar_far,
    lidar_width,
    lidar_height
)

# Configure noise filter (Gaussian noise with 5% standard deviation)
noise = csensor.ChNoiseGaussian(0.05)
lidar.SetNoise(noise)

# Enable saving frames to files
lidar.SetSaveFrames(True, 'lidar_frames/frame_')

# Initialize the sensor manager
sensor_mgr.Init()

# Visualization settings
sensor_mgr.SetVizType(chrono.render.RendererType_IRR)

# Setup visualization
system.SetupVisualization()
vis.SetCameraPosition(chrono.ChVectorD(10, 10, 10))
vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))

# Simulation loop
while system.GetChTime() < end_time:
    current_time = system.GetChTime()
    
    # Calculate current angle and position for the orbit
    angle = angular_speed * current_time
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    pos = chrono.ChVectorD(x, y, 0.0)
    
    # Compute direction towards origin
    dir = chrono.ChVectorD(-x, -y, 0.0).Get_normalized()
    up = chrono.ChVectorD(0, 0, 1)
    rot = chrono.Q_from_A_frame(dir, up)
    frame = chrono.ChFrameD(pos, rot)
    
    # Update sensor position and orientation
    sensor_mgr.UpdateSensor(lidar, frame, current_time)
    
    # Retrieve and print lidar buffer data
    buffer = lidar.GetBuffer()
    print(f"Time {current_time:.2f}: Lidar points = {len(buffer)}")
    if buffer:
        print(f"First point distance: {buffer[0].distance:.3f} m")
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Cleanup
chrono.GetLog() << "Simulation completed.\n"