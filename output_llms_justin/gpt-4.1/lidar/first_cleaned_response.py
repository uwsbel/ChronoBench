import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj")  
mesh.RepairDuplicateVertexes(1e-9)


mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetName("tri_mesh")
mesh_asset.SetColor(chrono.ChColor(0.4, 0.4, 0.8))
mesh_body.AddAsset(mesh_asset)


system.Add(mesh_body)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(2, 2, 2), chrono.ChColor(1, 1, 1), 500)



lidar_update_rate = 10.0  
lidar_h_fov = chrono.CH_C_PI  
lidar_v_fov = chrono.CH_C_PI / 4  
lidar_max_vert_angle = lidar_v_fov / 2
lidar_min_vert_angle = -lidar_v_fov / 2
lidar_max_horz_angle = lidar_h_fov / 2
lidar_min_horz_angle = -lidar_h_fov / 2
lidar_vert_samples = 16
lidar_horz_samples = 64
lidar_max_distance = 10.0  


lidar_offset = chrono.ChFrameD(chrono.ChVectorD(2, 0, 1), chrono.QUNIT)

lidar = sens.ChLidarSensor(
    mesh_body,          
    lidar_update_rate,  
    lidar_offset,       
    lidar_horz_samples, 
    lidar_vert_samples, 
    lidar_min_horz_angle,
    lidar_max_horz_angle,
    lidar_min_vert_angle,
    lidar_max_vert_angle,
    lidar_max_distance
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0)


lidar.PushFilter(sens.ChFilterLidarNoise(0.01))  


lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Visualization"))


lidar.PushFilter(sens.ChFilterSaveLidarPCD("lidar_output/"))


manager.AddSensor(lidar)


step_size = 1.0 / 100.0  
end_time = 2.0           
orbit_radius = 2.0
orbit_height = 1.0

print("Starting simulation...")

while system.GetChTime() < end_time:
    
    t = system.GetChTime()
    angle = 2 * math.pi * t / end_time  
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height

    
    pos = chrono.ChVectorD(x, y, z)
    target = chrono.ChVectorD(0, 0, 0)
    up = chrono.ChVectorD(0, 0, 1)
    dir = (target - pos).GetNormalized()
    
    rot = chrono.Q_from_AngAxis(math.atan2(y, x), chrono.ChVectorD(0, 0, 1))

    
    lidar.SetOffsetPose(chrono.ChFrameD(pos, rot))

    
    system.DoStepDynamics(step_size)
    manager.Update()

    
    lidar_data = lidar.GetMostRecentBuffer()
    if lidar_data is not None:
        
        points = lidar_data.GetPointCloud()
        print(f"Time {t:.2f}s: Lidar points = {points.shape[0] if hasattr(points, 'shape') else len(points)}")
    else:
        print(f"Time {t:.2f}s: No lidar data yet.")

print("Simulation complete.")