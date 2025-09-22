import pychrono as ch
import pychrono.sensor as sens
import math

# Create the Chrono physical system
system = ch.ChSystemNSC()
system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Add a mesh body to the system
mesh = ch.ChBodyEasyMesh(
    "../data/meshes/tri_mesh.obj",  # Path to the mesh file
    1000,  # Mesh density
    True,  # Automatically calculate mesh volume
    True,  # Automatically calculate mesh inertia
    ch.ChMaterialNSC()  # Material properties
)
mesh.SetPos(ch.ChVector3d(0, 0, 0))  # Set position of the mesh
mesh.SetFixed(True)  # Fix the mesh in place
system.Add(mesh)  # Add the mesh to the physical system

# Create a sensor manager
manager = sens.ChSensorManager(system)

# Create a lidar sensor
lidar = sens.ChLidarSensor(mesh,  # Attach lidar to the mesh
                           ch.ChFramed(ch.ChVector3d(0.0, 0, 1.7), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0))),  # Lidar frame
                           0.1,  # Lidar update rate
                           64,  # Number of horizontal samples
                           64,  # Number of vertical channels
                           1.0,  # Horizontal field of view
                           ch.SID(1.0, 1.0, 1.0),  # Lidar intensity (RGB)
                           100.0,  # Maximum lidar range
                           sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
                           0.004,  # Diameter of the lidar beam
                           0.002,  # Divergence angle of the lidar beam
                           sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAccess())  # Access raw lidar data
lidar.PushFilter(sens.ChFilterPCfromDepth())  # Convert depth data to point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(800, 600, 1.0, "Lidar PC data"))  # Visualize point cloud data
lidar.PushFilter(sens.ChFilterSaverPointCloud("../data/lidar_pc/"))  # Save point cloud data
manager.AddSensor(lidar)  # Add lidar to the sensor manager

# Simulation parameters
orbit_radius = 5  # Radius of the lidar orbit
orbit_speed = 0.5  # Speed of the lidar orbit
offset = 0  # Offset angle for the lidar orbit

# Simulation loop
for t in range(0, 1000):
    time = t * 0.02  # Calculate simulation time
    # Update lidar position to create an orbit
    lidar.SetPos(ch.ChVector3d(orbit_radius * math.cos(orbit_speed * time + offset), 
                               orbit_radius * math.sin(orbit_speed * time + offset), 
                               1.7))
    # Print lidar buffer data
    print(lidar.GetCurrentBuffer())
    # Advance simulation by one step
    system.DoStepDynamics(0.02)