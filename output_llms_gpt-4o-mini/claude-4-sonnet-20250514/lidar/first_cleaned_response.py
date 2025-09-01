import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np
import math
import os

def main():
    
    
    
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    
    
    
    
    
    
    mesh_body = chrono.ChBodyEasyMesh(
        "mesh.obj",  
        1000,        
        True,        
        True,        
        False,       
        True         
    )
    
    
    mesh_body.SetBodyFixed(True)
    
    
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    
    
    mesh_material = chrono.ChMaterialSurfaceNSC()
    mesh_material.SetFriction(0.5)
    mesh_material.SetRestitution(0.1)
    mesh_body.SetMaterialSurface(mesh_material)
    
    
    system.Add(mesh_body)
    
    
    
    
    
    
    manager = sens.ChSensorManager(system)
    
    
    manager.scene.AddPointLight(chrono.ChVectorF(0, 10, 0), chrono.ChVectorF(1, 1, 1), 100)
    manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChVectorF(1, 1, 1), 100)
    
    
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(5, 2, 0), chrono.Q_from_AngZ(0))
    
    
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_C_PI  
    vertical_fov_upper = chrono.CH_C_PI / 12  
    vertical_fov_lower = -chrono.CH_C_PI / 6  
    max_distance = 100
    
    
    lidar = sens.ChLidarSensor(
        mesh_body,                    
        30,                          
        offset_pose,                 
        horizontal_samples,          
        vertical_samples,           
        horizontal_fov,             
        vertical_fov_upper,         
        vertical_fov_lower,         
        max_distance               
    )
    
    
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)
    
    
    
    
    
    
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))
    
    
    lidar.PushFilter(sens.ChFilterLidarXYZI())
    
    
    if not os.path.exists("SENSOR_OUTPUT"):
        os.makedirs("SENSOR_OUTPUT")
    lidar.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/lidar"))
    
    
    manager.AddSensor(lidar)
    
    
    
    
    
    
    application = chronoirr.ChIrrApp(system, "PyChrono Lidar Simulation", chronoirr.dimension2du(1024, 768))
    application.AddTypicalSky()
    application.AddTypicalLogo()
    application.AddTypicalCamera(chronoirr.vector3df(8, 8, 8))
    application.AddTypicalLights()
    application.AssetBindAll()
    application.AssetUpdateAll()
    
    
    
    
    
    
    step_size = 1e-3
    sim_time = 0
    orbit_radius = 5
    orbit_height = 3
    orbit_speed = 0.5  
    
    application.SetTimestep(step_size)
    
    print("Starting simulation...")
    print("Lidar will orbit around the mesh while collecting data")
    
    while application.GetDevice().run():
        
        application.BeginScene()
        
        
        angle = orbit_speed * sim_time
        lidar_x = orbit_radius * math.cos(angle)
        lidar_z = orbit_radius * math.sin(angle)
        lidar_y = orbit_height
        
        
        look_direction = chrono.ChVectorD(-lidar_x, -lidar_y, -lidar_z)
        look_direction.Normalize()
        
        
        up_vector = chrono.ChVectorD(0, 1, 0)
        right_vector = look_direction % up_vector
        right_vector.Normalize()
        actual_up = right_vector % look_direction
        
        
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.Set_A_axis(right_vector, actual_up, -look_direction)
        rotation = chrono.ChQuaternionD()
        rotation.Set_A_matrix(rot_matrix)
        
        
        new_pose = chrono.ChFrameD(chrono.ChVectorD(lidar_x, lidar_y, lidar_z), rotation)
        lidar.SetOffsetPose(new_pose)
        
        
        manager.Update()
        
        
        lidar_data = lidar.GetMostRecentXYZIBuffer()
        if lidar_data.HasData():
            xyz_data = lidar_data.GetXYZI()
            print(f"Time: {sim_time:.2f}s, Lidar position: ({lidar_x:.2f}, {lidar_y:.2f}, {lidar_z:.2f})")
            print(f"Lidar data points: {len(xyz_data)}")
            
            
            valid_points = [(x, y, z, i) for x, y, z, i in xyz_data if not (x == 0 and y == 0 and z == 0)]
            if valid_points:
                print("Sample points (x, y, z, intensity):")
                for i, (x, y, z, intensity) in enumerate(valid_points[:5]):
                    print(f"  Point {i+1}: ({x:.3f}, {y:.3f}, {z:.3f}, {intensity:.3f})")
            print("-" * 50)
        
        
        application.DoStep()
        
        
        application.DrawAll()
        application.EndScene()
        
        
        sim_time += step_size
        
        
        if sim_time > 20.0:  
            break
    
    print("Simulation completed!")
    print(f"Lidar data saved to SENSOR_OUTPUT/ directory")

if __name__ == "__main__":
    main()