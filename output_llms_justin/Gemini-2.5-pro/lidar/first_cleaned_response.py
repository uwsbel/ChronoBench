import pychrono as chrono
import pychrono.sensor as sens
import os
import math
import numpy as np 




print("Creating Chrono system...")
physics_system = chrono.ChSystemNSC()
physics_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0)) 




print("Creating mesh body...")

mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0)) 
physics_system.Add(mesh_body)


obj_filename = "simple_pyramid.obj"


if not os.path.exists(obj_filename):
    print(f"Warning: '{obj_filename}' not found. Creating a dummy file.")
    with open(obj_filename, "w") as f:
        f.write("
        f.write("v 0.0 1.0 0.0\n")
        f.write("v -0.5 0.0 -0.5\n")
        f.write("v 0.5 0.0 -0.5\n")
        f.write("v 0.5 0.0 0.5\n")
        f.write("v -0.5 0.0 0.5\n")
        f.write("f 1 2 3\n")
        f.write("f 1 3 4\n")
        f.write("f 1 4 5\n")
        f.write("f 1 5 2\n")
        f.write("f 2 5 4\n")
        f.write("f 2 4 3\n")



trimesh_shape = chrono.ChTriangleMeshConnected()
try:
    trimesh_shape.LoadWavefrontMesh(obj_filename, False, True) 
    trimesh_shape.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1)) 
    
    
    vis_mesh = chrono.ChVisualShapeTriangleMesh()
    vis_mesh.SetMesh(trimesh_shape)
    vis_mesh.SetName("PyramidMesh")
    mesh_body.AddVisualShape(vis_mesh)

    
    
    
    
    
    

except Exception as e:
    print(f"Error loading OBJ file: {e}")
    print("Please ensure 'simple_pyramid.obj' exists and is valid.")
    exit()





print("Creating sensor manager...")
sensor_manager = sens.ChSensorManager(physics_system)
sensor_manager.SetVerbose(False) 




print("Creating Lidar sensor...")

update_rate = 10.0  
hfov = math.pi / 3  
vfov = math.pi / 6  
h_samples = 120      
v_samples = 60       
max_distance = 50.0  



initial_lidar_offset_pos = chrono.ChVectorD(0, 2.5, 0)
initial_lidar_offset_rot = chrono.Q_from_AngX(-math.pi / 2) 
initial_lidar_offset_pose = chrono.ChFrameD(initial_lidar_offset_pos, initial_lidar_offset_rot)

lidar = sens.ChLidarSensor(
    mesh_body,               
    update_rate,             
    initial_lidar_offset_pose, 
    h_samples,               
    v_samples,               
    hfov,                    
    vfov,                    
    max_distance,            
    
    
    
)
lidar.SetName("MyLidar")
lidar.SetLag(0) 
lidar.SetCollectionWindow(0) 




noise_model_xyzi = sens.ChNoiseNormalXYZI(0.0, 0.01, 0.0, 0.01, 0.0, 0.01, 0.0, 0.0) 
lidar.PushFilter(noise_model_xyzi)



lidar.PushFilter(sens.ChFilterVisualize("Lidar Point Cloud", 640, 480))



output_dir = "sensor_output/lidar_data/"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
lidar.PushFilter(sens.ChFilterSave(output_dir)) 

sensor_manager.AddSensor(lidar)
print("Lidar sensor added with filters.")




simulation_time = 10.0  
time_step = 1.0 / update_rate 
                               

current_time = 0.0


orbit_radius = 3.0
orbit_speed = math.pi / 5  
orbit_height = 1.5 

print(f"\nStarting simulation for {simulation_time} seconds...")
print(f"Lidar will orbit the mesh at radius {orbit_radius}m, height {orbit_height}m, speed {orbit_speed:.2f} rad/s.")
print("Press Ctrl+C in the terminal to stop early if visualization window is not responsive to closing.")

try:
    while current_time < simulation_time:
        
        angle = orbit_speed * current_time
        new_x = orbit_radius * math.cos(angle)
        new_z = orbit_radius * math.sin(angle)
        
        
        current_lidar_pos = chrono.ChVectorD(new_x, orbit_height, new_z)
        
        
        
        direction_to_origin = (chrono.ChVectorD(0,0.5,0) - current_lidar_pos).GetNormalized() 
        
        
        
        
        
        lidar_z_axis = direction_to_origin
        lidar_x_axis = chrono.Vcross(chrono.ChVectorD(0,1,0), lidar_z_axis).GetNormalized()
        lidar_y_axis = chrono.Vcross(lidar_z_axis, lidar_x_axis).GetNormalized()
        
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.SetCol(0, lidar_x_axis)
        rot_matrix.SetCol(1, lidar_y_axis)
        rot_matrix.SetCol(2, lidar_z_axis)
        
        current_lidar_rot = rot_matrix.GetQuaternion()
        
        new_offset_pose = chrono.ChFrameD(current_lidar_pos, current_lidar_rot)
        lidar.SetOffsetPose(new_offset_pose)

        
        sensor_manager.Update()

        
        physics_system.DoStepDynamics(time_step)
        
        
        
        
        xyzi_buffer = lidar.GetMostRecentBufferXYZI()
        if xyzi_buffer.HasData():
            
            
            point_cloud_data = xyzi_buffer.GetHostBufferXYZI() 
            num_points = len(point_cloud_data)
            print(f"Time: {current_time:.2f}s - Lidar captured {num_points} points.")
            
            
            
            
        else:
            print(f"Time: {current_time:.2f}s - No new Lidar data.")

        current_time += time_step

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")
finally:
    print("\nSimulation finished.")
    
    

    
    
    
    
    
    print(f"Lidar data saved in: {output_dir}")