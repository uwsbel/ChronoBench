import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import numpy as np
import os


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  
mesh_path = "PATH_TO_YOUR_MESH.obj"  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  



mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, False, True)  

mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.GetCollisionModel().ClearModel()
mesh_body.GetCollisionModel().AddTriangleMesh(mesh, False, False, chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
mesh_body.GetCollisionModel().BuildModel()
mesh_body.GetVisualModel().ClearModel()
mesh_body.GetVisualModel().AddTriangleMesh(mesh, False, False, chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
system.Add(mesh_body)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.SetSymbolScale(1.0)


sensor_manager = sensors.ChSensorManager(system)
system.Add(sensor_manager)


lidar = sensors.ChLidarSensor("lidar_sensor")
lidar.SetNumSamples(100)  
lidar.SetScanRate(10)     
lidar.SetMinRange(0.1)    
lidar.SetMaxRange(10)     
lidar.SetFov(360)         
lidar.SetBeamDivergence(0.1)  
lidar.SetSampleRes(0.01)  


noise_filter = sensors.ChGaussianNoiseFilter()
noise_filter.SetNoiseAmount(0.01)  
lidar.AddFilter(noise_filter)


lidar_visualization = sensors.ChLidarVisualization(sensor_manager)
lidar_visualization.SetLidar(lidar)
lidar_visualization.SetMaxPoints(10000)  
lidar_visualization.SetPointSize(2)      


data_saver = sensors.ChLidarDataSaver()
data_saver.SetLidar(lidar)
data_saver.SetSaveDirectory("lidar_data")  
data_saver.SetSaveFormat(sensors.ChLidarDataSaver.Format.CSV)  
data_saver.SetSaveRate(1)  


sensor_body = chrono.ChBody()
sensor_body.SetPos(chrono.ChVectorD(2, 0, 0))  
sensor_body.SetBodyFixed(False)
system.Add(sensor_body)

lidar.AttachBody(sensor_body)
lidar.SetPose(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))  
sensor_manager.AddSensor(lidar)


time_step = 0.01
end_time = 10.0
output_step = 0.1


current_time = 0
while current_time < end_time:
    
    angle = 2 * chrono.CH_PI * current_time / end_time
    radius = 3.0
    sensor_body.SetPos(chrono.ChVectorD(radius * np.cos(angle), radius * np.sin(angle), 1.0))

    
    sensor_body.SetRot(chrono.Q_from_AngAxis(-angle, chrono.ChVectorD(0, 0, 1)))

    
    sensor_manager.Update()

    
    if lidar.GetBuffer()->GetNumPoints() > 0:
        print(f"Time: {current_time:.2f}s - Points detected: {lidar.GetBuffer()->GetNumPoints()}")
        for i in range(lidar.GetBuffer()->GetNumPoints()):
            point = lidar.GetBuffer()->GetPoint(i)
            print(f"  Point {i}: Distance={point.dist:.3f}, Coords=({point.coords.x:.3f}, {point.coords.y:.3f}, {point.coords.z:.3f})")

    
    system.DoStepDynamics(time_step)
    current_time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    vis.Run()