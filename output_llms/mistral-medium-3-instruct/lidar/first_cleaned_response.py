import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os
import time





obj_file_path = "path/to/your/mesh.obj"


time_step = 0.01
sim_duration = 10.0
output_dir = "lidar_output"
os.makedirs(output_dir, exist_ok=True)


lidar_update_rate = 10  
lidar_range = 10.0      
lidar_resolution = 0.1  
lidar_fov = 360.0       
lidar_noise_std = 0.02  


orbit_radius = 5.0      
orbit_height = 2.0      
orbit_speed = 0.5       




chrono.SetChronoDataPath("path/to/chrono/data")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Triangular Mesh with Lidar")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 10))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.CameraVerticalDir::Z)




mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(obj_file_path, False, False)


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)  
mesh_body.GetCollisionModel().ClearModel()
mesh_body.GetCollisionModel().AddTriangleMesh(mesh, False, False)
mesh_body.GetCollisionModel().BuildModel()
mesh_body.SetCollide(True)


mesh_visual = chrono.ChTriangleMeshShape()
mesh_visual.SetMesh(mesh)
mesh_body.AddVisualShape(mesh_visual)


system.AddBody(mesh_body)




sensor_manager = sensors.ChSensorManager(system)


lidar = sensors.ChLidarSensor("lidar_sensor")
lidar.SetScanRate(lidar_update_rate)
lidar.SetRange(lidar_range)
lidar.SetResolution(lidar_resolution)
lidar.SetFOV(lidar_fov)


noise_filter = sensors.ChFilterGaussianNoise(lidar_noise_std)
lidar.AddFilter(noise_filter)


lidar_visual = sensors.ChVisualizationLidar(system, lidar)
lidar.AddVisualization(lidar_visual)


sensor_manager.AddSensor(lidar)




current_time = 0
step_number = 0

while current_time < sim_duration:
    
    angle = orbit_speed * current_time
    x = orbit_radius * np.cos(angle)
    z = orbit_radius * np.sin(angle)

    
    lidar_pos = chrono.ChVectorD(x, orbit_height, z)
    lidar_dir = mesh_body.GetPos() - lidar_pos  

    
    lidar.SetOffsetPose(chrono.ChFrameD(lidar_pos, chrono.Q_from_AngleAxis(angle, chrono.ChVectorD(0, 1, 0))))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    current_time += time_step
    step_number += 1

    
    sensor_manager.Update()

    
    if step_number % (int(1.0 / (time_step * lidar_update_rate))) == 0:
        print(f"\nTime: {current_time:.2f}s")
        print(f"Lidar position: ({lidar_pos.x:.2f}, {lidar_pos.y:.2f}, {lidar_pos.z:.2f})")

        
        buffer = lidar.GetMostRecentBuffer()
        if buffer:
            print(f"Number of points: {buffer.num_points}")
            print(f"First 5 points:")
            for i in range(min(5, buffer.num_points)):
                print(f"  Point {i}: range={buffer.points[i].range:.3f}m, angle={buffer.points[i].angle:.2f}°")

        
        if buffer:
            filename = os.path.join(output_dir, f"lidar_data_{current_time:.2f}.txt")
            with open(filename, 'w') as f:
                f.write(f"Time: {current_time:.2f}s\n")
                f.write(f"Lidar position: ({lidar_pos.x:.2f}, {lidar_pos.y:.2f}, {lidar_pos.z:.2f})\n")
                f.write("Point data:\n")
                f.write("range angle\n")
                for i in range(buffer.num_points):
                    f.write(f"{buffer.points[i].range:.3f} {buffer.points[i].angle:.2f}\n")
            print(f"Saved lidar data to {filename}")




print("\nSimulation completed.")