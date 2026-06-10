import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math
import os




print("Initializing PyChrono simulation environment...")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))




print("Loading triangular mesh from Wavefront .obj file...")


mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetMass(1.0)


tri_mesh = chrono.ChTriangleMeshConnected()


obj_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")


if not os.path.exists(obj_file):
    print(f"  Warning: '{obj_file}' not found. Trying alternative path...")
    obj_file = chrono.GetChronoDataFile("sensor/offroad/ground.obj")

if not os.path.exists(obj_file):
    print("  Warning: Default mesh not found. Creating a simple procedural mesh...")
    
    tri_mesh.AddTriangle(
        chrono.ChVector3d(-10, -10, 0),
        chrono.ChVector3d( 10, -10, 0),
        chrono.ChVector3d( 10,  10, 0)
    )
    tri_mesh.AddTriangle(
        chrono.ChVector3d(-10, -10, 0),
        chrono.ChVector3d( 10,  10, 0),
        chrono.ChVector3d(-10,  10, 0)
    )
    
    h = 2.0
    vertices = [
        chrono.ChVector3d(-2, -2, 0), chrono.ChVector3d( 2, -2, 0),
        chrono.ChVector3d( 2,  2, 0), chrono.ChVector3d(-2,  2, 0),
        chrono.ChVector3d(-2, -2, h), chrono.ChVector3d( 2, -2, h),
        chrono.ChVector3d( 2,  2, h), chrono.ChVector3d(-2,  2, h),
    ]
    faces = [(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),
             (3,0,4),(3,4,7),(4,5,6),(4,6,7)]
    for f in faces:
        tri_mesh.AddTriangle(vertices[f[0]], vertices[f[1]], vertices[f[2]])
    print("  Procedural mesh created successfully.")
else:
    tri_mesh.LoadWavefrontMesh(obj_file, True, True)
    print(f"  Mesh loaded from: {obj_file}")


vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh.SetMesh(tri_mesh)
vis_mesh.SetName("SceneMesh")
vis_mesh.SetMutable(False)


mat = chrono.ChVisualMaterial()
mat.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.6))
mat.SetAmbientColor(chrono.ChColor(0.2, 0.2, 0.2))
mat.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
vis_mesh.material_list.append(mat)

mesh_body.AddVisualShape(vis_mesh)
sys.Add(mesh_body)
print("  Fixed mesh body added to simulation.")




print("Setting up sensor manager...")
manager = sens.ChSensorManager(sys)


intensity = 1.5
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 50),
    chrono.ChColor(intensity, intensity, intensity),
    5000
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 10, 20),
    chrono.ChColor(intensity * 0.7, intensity * 0.7, intensity * 0.7),
    3000
)
manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))




print("Configuring lidar sensor parameters...")


update_rate      = 10.0              
lag              = 0.0               
collection_time  = 1.0 / update_rate 


horizontal_samples = 800    
vertical_samples   = 16     


horizontal_fov  = 2.0 * math.pi     
max_vert_angle  =  math.pi / 12     
min_vert_angle  = -math.pi / 6      


max_range = 100.0  


beam_shape    = sens.LidarBeamShape_RECTANGULAR
sample_radius = 2
divergence_h  = 0.003  
divergence_v  = 0.003  
return_mode   = sens.LidarReturnMode_STRONGEST_RETURN




ORBIT_RADIUS = 12.0    
ORBIT_HEIGHT = 3.0     
ORBIT_SPEED  = 0.25    

initial_angle = 0.0
init_x = ORBIT_RADIUS * math.cos(initial_angle)
init_y = ORBIT_RADIUS * math.sin(initial_angle)
init_z = ORBIT_HEIGHT


init_rot = chrono.QuatFromAngleAxis(
    initial_angle + math.pi,
    chrono.ChVector3d(0, 0, 1)
)
offset_pose = chrono.ChFramed(chrono.ChVector3d(init_x, init_y, init_z), init_rot)




print("Creating lidar sensor...")
lidar = sens.ChLidarSensor(
    mesh_body,           
    update_rate,         
    offset_pose,         
    horizontal_samples,  
    vertical_samples,    
    horizontal_fov,      
    max_vert_angle,      
    min_vert_angle,      
    max_range,           
    beam_shape,          
    sample_radius,       
    divergence_h,        
    divergence_v,        
    return_mode          
)

lidar.SetName("OrbitingLidar")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)




print("Attaching lidar filter pipeline...")



lidar.PushFilter(sens.ChFilterPCfromDepth())
print("  [1] ChFilterPCfromDepth      - Raw depth -> XYZ point cloud")



lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.0, 0.01, 0.003, 0.003))
print("  [2] ChFilterLidarNoiseXYZI   - Gaussian noise on point cloud")



lidar.PushFilter(sens.ChFilterVisualizePointCloud(960, 600, 0.8, "Lidar Point Cloud"))
print("  [3] ChFilterVisualizePointCloud - Real-time point cloud visualization")



save_dir = "output/lidar_ptcloud/"
os.makedirs(save_dir, exist_ok=True)
lidar.PushFilter(sens.ChFilterSavePtCloud(save_dir))
print(f"  [4] ChFilterSavePtCloud      - Saving to '{save_dir}'")



lidar.PushFilter(sens.ChFilterXYZIAccess())
print("  [5] ChFilterXYZIAccess       - Python buffer access enabled")


manager.AddSensor(lidar)
print("  Lidar sensor added to manager.")




step_size = 1e-3     
sim_time  = 0.0      
end_time  = 30.0     

step_number      = 0
total_scans      = 0
last_update_time = -1.0
lidar_period     = 1.0 / update_rate

print("\n" + "=" * 65)
print("  SIMULATION PARAMETERS")
print("=" * 65)
print(f"  Duration          : {end_time:.1f} s")
print(f"  Step size         : {step_size*1000:.1f} ms")
print(f"  Lidar update rate : {update_rate:.1f} Hz")
print(f"  Orbit radius      : {ORBIT_RADIUS:.1f} m")
print(f"  Orbit height      : {ORBIT_HEIGHT:.1f} m")
print(f"  Orbit speed       : {ORBIT_SPEED:.2f} rad/s")
print(f"  Horizontal samples: {horizontal_samples}")
print(f"  Vertical channels : {vertical_samples}")
print(f"  Max range         : {max_range:.1f} m")
print("=" * 65)
print("  Starting simulation loop...\n")

while sim_time < end_time:

    
    
    
    angle = ORBIT_SPEED * sim_time

    lidar_x = ORBIT_RADIUS * math.cos(angle)
    lidar_y = ORBIT_RADIUS * math.sin(angle)
    lidar_z = ORBIT_HEIGHT

    
    lidar_rot = chrono.QuatFromAngleAxis(
        angle + math.pi,
        chrono.ChVector3d(0, 0, 1)
    )

    
    new_pose = chrono.ChFramed(
        chrono.ChVector3d(lidar_x, lidar_y, lidar_z),
        lidar_rot
    )
    lidar.SetOffsetPose(new_pose)

    
    
    
    manager.Update()

    
    
    
    if sim_time - last_update_time >= lidar_period - (step_size * 0.5):
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()

        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            total_scans += 1

            
            
            num_channels = xyzi_data.shape[0]
            num_h_samples = xyzi_data.shape[1]
            total_points = num_channels * num_h_samples

            
            ranges = np.sqrt(
                xyzi_data[:, :, 0]**2 +
                xyzi_data[:, :, 1]**2 +
                xyzi_data[:, :, 2]**2
            )
            valid_mask  = ranges > 0.01
            valid_count = int(np.sum(valid_mask))
            avg_range   = float(np.mean(ranges[valid_mask])) if valid_count > 0 else 0.0
            avg_intensity = float(np.mean(xyzi_data[:, :, 3][valid_mask])) if valid_count > 0 else 0.0

            
            sample_str = "N/A"
            if valid_count > 0:
                valid_indices = np.argwhere(valid_mask)
                mid_idx = valid_indices[len(valid_indices) // 2]
                pt = xyzi_data[mid_idx[0], mid_idx[1], :]
                sample_str = f"[{pt[0]:+.3f}, {pt[1]:+.3f}, {pt[2]:+.3f}, i={pt[3]:.3f}]"

            print(f"[t={sim_time:6.3f}s | Scan 
            print(f"  Lidar position  : ({lidar_x:+7.3f}, {lidar_y:+7.3f}, {lidar_z:+5.3f}) m")
            print(f"  Orbit angle     : {math.degrees(angle):6.1f} deg")
            print(f"  Buffer shape    : {xyzi_data.shape}  "
                  f"[channels x samples x (X,Y,Z,I)]")
            print(f"  Total points    : {total_points:6d} | "
                  f"Valid hits: {valid_count:6d} ({100*valid_count/total_points:.1f}%)")
            print(f"  Avg range       : {avg_range:.3f} m | "
                  f"Avg intensity: {avg_intensity:.4f}")
            print(f"  Sample point    : {sample_str}")
            print()

            last_update_time = sim_time

        else:
            
            print(f"[t={sim_time:6.3f}s] Waiting for lidar buffer to populate...")
            last_update_time = sim_time

    
    
    
    sys.DoStepDynamics(step_size)
    sim_time  += step_size
    step_number += 1




print("=" * 65)
print("  SIMULATION COMPLETE")
print("=" * 65)
print(f"  Total simulation time : {end_time:.1f} s")
print(f"  Total steps executed  : {step_number}")
print(f"  Total lidar scans     : {total_scans}")
print(f"  Point cloud data saved to: '{save_dir}'")
print("=" * 65)