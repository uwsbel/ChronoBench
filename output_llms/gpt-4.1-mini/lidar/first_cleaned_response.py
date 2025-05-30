import pychrono as chrono
import pychrono.sensor as sens
import os
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  




mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetCollide(False)


mesh_obj_path = os.path.join(chrono.GetChronoDataPath(), "sensor", "objects", "terrain_simple.obj")



mesh_asset = chrono.ChTriangleMeshShape()
tm = chrono.ChTriangleMeshConnected()
if not tm.LoadWavefrontMesh(mesh_obj_path, True, True):
    raise Exception(f"Failed to load mesh from {mesh_obj_path}")
mesh_asset.SetMesh(tm)
mesh_asset.SetName("mesh_visualization")
mesh_asset.SetStatic(True)
mesh_asset.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

mesh_body.AddAsset(mesh_asset)
system.Add(mesh_body)




sensor_manager = sens.SensorManager(system)
sensor_manager.SetVerbose(False)
update_rate = 60  
sensor_manager.SetSensorUpdateRate(update_rate)
sensor_manager.SetWindowHandle(800, 600)


lidar = sens.ChLidarSensor(mesh_body)


lidar.SetName("lidar_sensor")
lidar.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.Q_from_AngY(math.radians(90))))
lidar.SetScanWindow(math.radians(180))  
lidar.SetNumberOfVerticalChannels(3)
lidar.SetVerticalFOV(math.radians(10))  
lidar.SetHorizontalResolution(200)  
lidar.SetMaxDistance(10)  


output_folder = "lidar_output"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)
lidar.SetFilePathCSV(output_folder + "/lidar_scan.csv")
lidar.SetFilePathPCD(output_folder + "/lidar_scan.pcd")


gauss_noise = sens.ChLidarNoiseGaussian(0.1)
lidar.AddFilter(gauss_noise)


viz_filter = sens.ChFilterVisualizePointCloud()
lidar.AddFilter(viz_filter)


csvfilter = sens.ChFilterSaveCSV()
lidar.AddFilter(csvfilter)

pcdfilter = sens.ChFilterSavePCD()
lidar.AddFilter(pcdfilter)

sensor_manager.AddSensor(lidar)






simulation_time = 10  
step_size = 1.0 / 500  


orbit_radius = 1.0
orbit_freq = 0.5  


print_interval = 1.0 / update_rate  

num_steps = int(simulation_time / step_size)
next_print_time = 0

for step in range(num_steps):
    time = step * step_size

    
    system.DoStepDynamics(step_size)

    
    angle = 2 * math.pi * orbit_freq * time
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = 0.2  

    
    
    lidar.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(x, y, z),
                                       chrono.Q_from_AngAxis(angle + math.pi/2, chrono.ChVectorD(0, 0, 1))))

    
    sensor_manager.Update()

    
    if time >= next_print_time:
        next_print_time += print_interval
        
        points = lidar.GetPointCloudBuffer()
        if points is not None:
            print(f"Time {time:.3f} s: Lidar points count = {len(points)}")
            max_print = min(10, len(points))
            for i in range(max_print):
                p = points[i]
                print(f"  Point[{i}] = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) depth={p.z}")
        else:
            print(f"Time {time:.3f} s: No lidar points available yet.")

print("Simulation complete.")