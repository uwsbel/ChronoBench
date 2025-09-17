import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono system and Irrlicht visualization
system = chrono.ChSystemNSC()
application = chronoirr.ChIrrApp(system, 'PyChrono Lidar Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 3, 6), chronoirr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()

# Load the triangular mesh from .obj file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('path_to_mesh.obj', True, True)  # Replace with actual mesh path

# Create a fixed body for the mesh
body = chrono.ChBody()
body.SetBodyFixed(True)
body.GetCollisionModel().ClearModel()
body.GetCollisionModel().AddTriangleMesh(mesh, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1), 0.001)
body.SetCollide(False)

# Add visualization for the mesh
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh)
vis_shape.SetOpacity(1.0)
body.AddVisualShape(vis_shape)

system.Add(body)

# Create sensor manager and lidar sensor
manager = sens.ChSensorManager(system)

lidar = sens.ChLidarSensor(
    body,                    # Parent body
    100,                   # Update rate in Hz
    chrono.ChFrameD(),     # Initial offset (will be updated dynamically)
    360,                   # Horizontal samples
    30,                    # Vertical samples
    chrono.CH_C_PI,        # Horizontal field of view (180 degrees)
    chrono.CH_C_PI / 4,    # Vertical field of view (45 degrees)
    100,                   # Maximum distance
    sens.LidarBeamShape_RECTANGULAR,
    2,                     # Sample radius
    0.003,                 # Divergence angle (horizontal)
    0.003                  # Divergence angle (vertical)
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1 / 100)

# Add filters to the lidar
lidar.AddFilter(sens.ChFilterNoiseLidarNormal(0.01))  # Add Gaussian noise
lidar.AddFilter(sens.ChFilterVisual())               # Enable visualization
lidar.AddFilter(sens.ChFilterPCwrite())              # Enable point cloud saving

manager.AddSensor(lidar)

# Simulation loop parameters
time_step = 0.01
time = 0
orbit_radius = 5
orbit_height = 2
omega = 0.5  # Angular velocity (rad/s)

# Run the simulation loop
while application.GetDevice().run():
    # Calculate new lidar position in orbit
    theta = omega * time
    x = orbit_radius * math.cos(theta)
    z = orbit_radius * math.sin(theta)
    y = orbit_height
    lidar_pos = chrono.ChVectorD(x, y, z)
    
    # Compute rotation to look at the origin
    dir_to_origin = -lidar_pos.GetNormalized()
    rot = chrono.ChQuaternionFromTwoVectors(chrono.ChVectorD(1, 0, 0), dir_to_origin)
    
    # Update lidar's offset pose
    lidar.SetOffsetPose(chrono.ChFrameD(lidar_pos, rot))
    
    # Advance the simulation
    system.DoStepDynamics(time_step)
    manager.Update()
    
    # Print lidar data if available
    if lidar.HasData():
        data = lidar.GetMostRecentData()
        point_cloud = data.GetPointcloud()
        print(f"Time: {time:.2f}, Point cloud size: {point_cloud.size()}")
        if point_cloud.size() > 0:
            first_point = point_cloud[0]
            print(f"  First point: ({first_point.x:.2f}, {first_point.y:.2f}, {first_point.z:.2f})")
    
    # Update Irrlicht visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    time += time_step