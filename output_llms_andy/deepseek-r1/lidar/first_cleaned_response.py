import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import numpy as np
import os


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_path = chrono.GetChronoDataFile("models/cube.obj")  
mesh_visual = chrono.ChVisualShapeModelFile(mesh_path)
mesh_body.AddVisualShape(mesh_visual)
system.Add(mesh_body)


manager = sens.ChSensorManager(system)


offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0))  
lidar = sens.ChLidarSensor(
    parent=mesh_body,         
    update_rate=10,           
    offset_pose=offset_pose,
    horizontal_samples=90,    
    vertical_samples=5,       
    horizontal_fov=90,        
    vertical_fov=10,          
    max_vertical_angle=5,     
    min_vertical_angle=-5,    
    max_distance=100          
)


noise_model = sens.ChNoiseNormal(mean=0, stdev=0.01)
lidar.SetNoiseModel(noise_model)


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterAccess())
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))


lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0))


manager.AddSensor(lidar)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 2))
vis.AddTypicalLights()


orbit_radius = 3.0
orbit_height = 1.0
angular_speed = 0.1  
time_step = 0.01
current_time = 0.0

print("Starting simulation. Press ESC to exit...")

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    angle = angular_speed * current_time
    x = orbit_radius * np.cos(angle)
    y = orbit_radius * np.sin(angle)
    lidar.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(x, y, orbit_height)))

    
    manager.Update()

    
    buffer = lidar.GetMostRecentBuffer()
    if buffer.HasData():
        point_cloud = buffer.GetPointCloud()
        print(f"Time: {current_time:.2f}s | Points: {len(point_cloud)}")
        if len(point_cloud) > 0:
            print(f"Sample point: {point_cloud[0]}")

    
    system.DoStepDynamics(time_step)
    current_time += time_step