import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Simulation with OBJ Mesh")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChVectorD(0, 0, 0))


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)


mesh_path = "path/to/your/mesh.obj"  
if not os.path.exists(mesh_path):
    raise FileNotFoundError(f"Mesh file not found: {mesh_path}")

mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, True, True)

mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(mesh)
mesh_asset.SetName("OBJ_Mesh")
mesh_body.AddAsset(mesh_asset)


sensor_manager = sensors.ChSensorManager(system)
lidar = sensors.ChLidarSensor()


lidar.SetName("Lidar_Sensor")
lidar.SetUpdateRate(30)  
lidar.SetScanRate(100)   
lidar.SetVerticalResolution(32)  
lidar.SetVerticalFOV(30)  
lidar.SetHorizontalFOV(360)  
lidar.SetMinRange(0.1)    
lidar.SetMaxRange(50)     


noise_model = sensors.ChNoiseGaussian(0.01)  
lidar.AddNoiseModel(noise_model)


lidar_vis = sensors.ChLidarVisualization(system, lidar)
sensor_manager.AddSensor(lidar)


lidar_body = chrono.ChBody()
system.Add(lidar_body)
lidar_body.SetPos(chrono.ChVectorD(0, 0, 0))
lidar_body.SetBodyFixed(False)  


lidar.SetBody(lidar_body)


lidar_data = sensors.ChLidarData()
lidar.SetData(lidar_data)
lidar_data.SetSaveToFile(True)
lidar_data.SetFileName("lidar_data.txt")


time_step = 0.01
end_time = 10.0
current_time = 0


while vis.Run() and current_time < end_time:
    
    angle = current_time * 0.5  
    radius = 2.0  
    lidar_body.SetPos(chrono.ChVectorD(
        radius * np.cos(angle),
        radius * np.sin(angle),
        1.0  
    ))

    
    lidar_body.SetRot(chrono.Q_from_AngAxis(angle + np.pi, chrono.ChVectorD(0, 0, 1)))

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sensor_manager.Update()

    
    if lidar_data.GetNumPoints() > 0:
        print(f"Time: {current_time:.2f}s - Points detected: {lidar_data.GetNumPoints()}")
        
        for i in range(min(5, lidar_data.GetNumPoints())):
            point = lidar_data.GetPoint(i)
            print(f"  Point {i}: Distance={point.distance:.2f}m, Intensity={point.intensity:.2f}")

    
    system.DoStepDynamics(time_step)
    current_time += time_step


lidar_data.SaveToFile()