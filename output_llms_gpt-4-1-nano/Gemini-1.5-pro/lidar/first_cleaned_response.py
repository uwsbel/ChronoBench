import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))


time_step = 1e-3
system.SetStep(time_step)




chrono.SetChronoDataPath("./data/")


mesh_asset = chrono.ChAsset()
mesh_asset.AppendAsset(chrono.ChObjShapeFile("./sensor_test_assets/house.obj"))
mesh_body = chrono.ChBodyEasyMesh(
    mesh_asset,  
    1000,        
    True,        
    True,        
    False        
)
system.Add(mesh_body)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)  




manager = sens.ChSensorManager(system)


lidar = sens.ChLidarSensor(
    mesh_body,              
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2)),  
    1.0,                   
    chrono.ChVectorD(0, 1, 0), 
    50,                   
    chrono.CH_C_DEG_TO_RAD * 20.0, 
    50.0                 
)
lidar.SetName("Lidar Sensor")

lidar.SetShowRays(True)
lidar.SetRaysVisualizationLength(20)
lidar.SetPointSize(0.01)

lidar.SetNoiseModel(sens.ChLidarSensor.NoiseModel.GAUSSIAN)
lidar.SetGaussianNoise(0.005)

lidar.SetSaveHorizontalRays(True)
lidar.SetSaveVerticalRays(False)

manager.AddSensor(lidar)





t_end = 10.0
time = 0

while time < t_end:
    
    orbit_radius = 5.0
    orbit_speed = 0.5
    angle = orbit_speed * time
    lidar.SetOffsetPos(chrono.ChVectorD(orbit_radius * math.cos(angle), 0, orbit_radius * math.sin(angle)))

    
    manager.Update()
    system.DoStepDynamics(time_step)

    
    print(f"Time: {time:.2f}")
    print(f"Lidar data buffer size: {lidar.GetPointCloud().size()}")

    time += time_step




manager.ExportData("./sensor_output/test_output")