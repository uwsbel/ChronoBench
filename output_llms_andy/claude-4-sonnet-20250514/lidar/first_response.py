import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np
import math
import os

def main():
    # ---------------------------------------------------------------------
    # Create the simulation system and set parameters
    # ---------------------------------------------------------------------
    
    # Create the Chrono::Engine physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # Set the default outward/inward shape margins for collision detection
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    
    # ---------------------------------------------------------------------
    # Create a triangular mesh body from .obj file
    # ---------------------------------------------------------------------
    
    # Create a fixed body for the mesh
    mesh_body = chrono.ChBodyEasyMesh(
        "mesh.obj",  # You can replace this with your .obj file path
        1000,        # density
        True,        # visualization mesh
        True,        # collision mesh
        False,       # mesh is not closed
        True         # compute mass and inertia automatically
    )
    
    # Set the mesh body as fixed (stationary)
    mesh_body.SetBodyFixed(True)
    
    # Position the mesh at the origin
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    
    # Set material properties
    mesh_material = chrono.ChMaterialSurfaceNSC()
    mesh_material.SetFriction(0.5)
    mesh_material.SetRestitution(0.1)
    mesh_body.SetMaterialSurface(mesh_material)
    
    # Add the mesh body to the system
    system.Add(mesh_body)
    
    # ---------------------------------------------------------------------
    # Create sensor manager and lidar sensor
    # ---------------------------------------------------------------------
    
    # Create sensor manager
    manager = sens.ChSensorManager(system)
    
    # Set the background color for the sensor camera
    manager.scene.AddPointLight(chrono.ChVectorF(0, 10, 0), chrono.ChVectorF(1, 1, 1), 100)
    manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChVectorF(1, 1, 1), 100)
    
    # Create a lidar sensor
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(5, 2, 0), chrono.Q_from_AngZ(0))
    
    # Lidar parameters
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
    vertical_fov_upper = chrono.CH_C_PI / 12  # 15 degrees up
    vertical_fov_lower = -chrono.CH_C_PI / 6  # 30 degrees down
    max_distance = 100
    
    # Create the lidar sensor
    lidar = sens.ChLidarSensor(
        mesh_body,                    # body to attach to
        30,                          # update rate in Hz
        offset_pose,                 # offset pose
        horizontal_samples,          # horizontal samples
        vertical_samples,           # vertical samples
        horizontal_fov,             # horizontal field of view
        vertical_fov_upper,         # vertical field of view (upper)
        vertical_fov_lower,         # vertical field of view (lower)
        max_distance               # max distance
    )
    
    # Set lidar visualization type
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)
    
    # ---------------------------------------------------------------------
    # Add filters to the lidar sensor
    # ---------------------------------------------------------------------
    
    # Add noise filter
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    # Add visualization filter
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))
    
    # Add access filter to get the data
    lidar.PushFilter(sens.ChFilterLidarXYZI())
    
    # Add save filter (saves to file)
    if not os.path.exists("SENSOR_OUTPUT"):
        os.makedirs("SENSOR_OUTPUT")
    lidar.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/lidar"))
    
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)
    
    # ---------------------------------------------------------------------
    # Create the Irrlicht visualization
    # ---------------------------------------------------------------------
    
    # Create the Irrlicht application
    application = chronoirr.ChIrrApp(system, "PyChrono Lidar Simulation", chronoirr.dimension2du(1024, 768))
    application.AddTypicalSky()
    application.AddTypicalLogo()
    application.AddTypicalCamera(chronoirr.vector3df(8, 8, 8))
    application.AddTypicalLights()
    application.AssetBindAll()
    application.AssetUpdateAll()
    
    # ---------------------------------------------------------------------
    # Simulation loop
    # ---------------------------------------------------------------------
    
    # Simulation parameters
    step_size = 1e-3
    sim_time = 0
    orbit_radius = 5
    orbit_height = 3
    orbit_speed = 0.5  # radians per second
    
    application.SetTimestep(step_size)
    
    print("Starting simulation...")
    print("Lidar will orbit around the mesh while collecting data")
    
    while application.GetDevice().run():
        # Begin scene
        application.BeginScene()
        
        # Update lidar position in orbit around the mesh
        angle = orbit_speed * sim_time
        lidar_x = orbit_radius * math.cos(angle)
        lidar_z = orbit_radius * math.sin(angle)
        lidar_y = orbit_height
        
        # Calculate orientation to look at the mesh center
        look_direction = chrono.ChVectorD(-lidar_x, -lidar_y, -lidar_z)
        look_direction.Normalize()
        
        # Create rotation to point towards center
        up_vector = chrono.ChVectorD(0, 1, 0)
        right_vector = look_direction % up_vector
        right_vector.Normalize()
        actual_up = right_vector % look_direction
        
        # Create rotation matrix
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.Set_A_axis(right_vector, actual_up, -look_direction)
        rotation = chrono.ChQuaternionD()
        rotation.Set_A_matrix(rot_matrix)
        
        # Update lidar pose
        new_pose = chrono.ChFrameD(chrono.ChVectorD(lidar_x, lidar_y, lidar_z), rotation)
        lidar.SetOffsetPose(new_pose)
        
        # Update sensor manager
        manager.Update()
        
        # Get lidar data and print information
        lidar_data = lidar.GetMostRecentXYZIBuffer()
        if lidar_data.HasData():
            xyz_data = lidar_data.GetXYZI()
            print(f"Time: {sim_time:.2f}s, Lidar position: ({lidar_x:.2f}, {lidar_y:.2f}, {lidar_z:.2f})")
            print(f"Lidar data points: {len(xyz_data)}")
            
            # Print some sample points (first 5 valid points)
            valid_points = [(x, y, z, i) for x, y, z, i in xyz_data if not (x == 0 and y == 0 and z == 0)]
            if valid_points:
                print("Sample points (x, y, z, intensity):")
                for i, (x, y, z, intensity) in enumerate(valid_points[:5]):
                    print(f"  Point {i+1}: ({x:.3f}, {y:.3f}, {z:.3f}, {intensity:.3f})")
            print("-" * 50)
        
        # Advance the simulation
        application.DoStep()
        
        # Render the scene
        application.DrawAll()
        application.EndScene()
        
        # Update simulation time
        sim_time += step_size
        
        # Optional: break after certain time for demonstration
        if sim_time > 20.0:  # Run for 20 seconds
            break
    
    print("Simulation completed!")
    print(f"Lidar data saved to SENSOR_OUTPUT/ directory")

if __name__ == "__main__":
    main()