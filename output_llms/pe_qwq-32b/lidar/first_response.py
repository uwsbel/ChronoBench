import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postproc
import math

# Initialize system and visualization
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Mesh Simulation")
vis.AttachSystem(system)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chronoirr.command.GetCamera())
vis.AddTypicalLights()
vis.Initialize()
vis.GetCamera().SetPos(chrono.ChVectorD(3, 3, 3))
vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))

# Load mesh from .obj file
mesh_path = chrono.GetChronoDataFile("meshes/cube.obj")  # Replace with actual .obj path
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(chrono.ChTriangleMesh())
mesh_shape.GetMesh().LoadWavefrontObj(mesh_path)

# Create fixed mesh body
fixed_body = chrono.ChBodyEasyMesh(0, 0, True, mesh_shape.GetMesh())
fixed_body.SetPos(chrono.ChVectorD(0, 0, 0))
fixed_body.SetBodyFixed(True)
system.Add(fixed_body)
fixed_body.AddAsset(mesh_shape)

# Lidar sensor setup
sensor_mgr = postproc.ChSensorManager(system)

lidar = postproc.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetRange(50)  # Maximum range
lidar.SetHorizontalResolution(0.5)  # Degrees
lidar.SetVerticalResolution(0.5)
lidar.SetFOVHorizontal(360)
lidar.SetFOVVertical(30)
lidar.SetNoiseFilter(postproc.ChLidarNoiseFilterGaussian(0.05))
lidar.SetSaveToFile(True, "lidar_data.csv")
lidar.SetVisualizationType(postproc.LASER_VIZ_BEAMS)

# Initial lidar position (orbit parameters)
orbit_radius = 5.0
orbit_height = 2.0
time = 0.0
angular_speed = 0.5  # Radians per second

# Update lidar position each step
def update_lidar_position(time):
    angle = angular_speed * time
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height
    return chrono.ChFrameD(chrono.ChVectorD(x, y, z), chrono.Q_from_AngZ(angle))

# Attach sensor to moving frame
sensor_frame = chrono.ChFrameD()
lidar.SetPositionFrame(update_lidar_position(0))
sensor_mgr.AddSensor(lidar)

# Simulation loop
time_step = 0.01
while vis.Run():
    time += time_step
    
    # Update lidar position
    sensor_frame.SetPos(update_lidar_position(time).GetPos())
    lidar.SetPositionFrame(sensor_frame)
    
    system.DoStepDynamics(time_step)
    sensor_mgr.Update()
    
    # Print lidar data
    buffer = lidar.GetBuffer()
    print(f"Step {time:.2f}s | Points: {len(buffer)}")
    if buffer:
        print("Sample point:", buffer[0])
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()