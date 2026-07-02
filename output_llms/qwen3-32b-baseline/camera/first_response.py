import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irr as irr
import math

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, 0, 0))  # No gravity

# Create fixed body with triangular mesh
mesh_body = chrono.ChBodyFixed()
mesh_body.SetName("MeshBody")
system.Add(mesh_body)

# Load the .obj mesh
mesh_shape = chrono.ChTriangleMeshConnected()
mesh_shape.LoadWavefrontMesh("mesh.obj", False, False)
mesh_visual = chrono.ChVisualShapeTriangleMesh(mesh_shape, False, False)
mesh_body.AddAsset(mesh_visual)

# Set up sensor manager
sensor_manager = sens.ChSensorManager(system)

# Create camera sensor
camera = sens.ChCameraSensor(
    mesh_body,  # parent
    30,  # update rate (Hz)
    chrono.ChFrameD(chrono.ChVectorD(2, 2, 2)),  # initial offset
    640, 480  # resolution
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(1 / 30)

# Add noise and visualization filters
camera.PushFilter(sens.ChFilterCameraNoise())
camera.PushFilter(sens.ChFilterAccessBuffer(sens.ChFilterBufferType.RGBA))
camera.PushFilter(sens.ChFilterVisualize(640, 480))

sensor_manager.AddSensor(camera)

# Set up Irrlicht visualization
application = irr.ChIrrApp(system, "PyChrono Simulation", irr.dimension2du(800, 600))
application.AddTypicalCamera(chronoirr.vector3df(2, 2, 2))
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetTargetBody(mesh_body)
application.SetTimestep(0.001)  # 1 ms step

# Simulation parameters
orbit_radius = 2
theta = 0
angular_velocity = 0.05  # rad/s
time_step = 0.001
simulation_time = 10.0

# Simulation loop
time = 0
while time < simulation_time:
    # Update camera position/orientation
    pos_x = orbit_radius * math.cos(theta)
    pos_z = orbit_radius * math.sin(theta)
    pos = chrono.ChVectorD(pos_x, 0, pos_z)
    
    # Compute direction to look at origin
    dir = -pos
    dir.Normalize()
    
    # Compute rotation
    rot = chrono.ChQuaternionD()
    rot.SetFromDirs(chrono.ChVectorD(0, 0, -1), dir)
    
    camera.SetOffsetFrame(chrono.ChFrameD(pos, rot))
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    sensor_manager.Update()
    
    # Get and print buffer data
    buffer = camera.GetMostRecentBuffer()
    if buffer:
        print(f"Time: {time:.3f}")
        data = buffer.GetRGBAidata()
        print(f"First 10 pixel values: {data[:10]}")
    
    # Render scene
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    time += time_step
    theta += angular_velocity * time_step