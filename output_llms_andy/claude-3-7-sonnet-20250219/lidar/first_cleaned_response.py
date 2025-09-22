import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math
import os




chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




mesh_body = chrono.ChBodyEasyMesh(
    filename="mesh.obj",  
    mass=1.0,
    visualize=True,
    collide=True)
mesh_body.SetBodyFixed(True)  
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)




manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(100, 100, 100), chrono.ChColor(1, 1, 1), 1000.0)


lidar_pos = chrono.ChFrameD(chrono.ChVectorD(0, 2, 5), chrono.QUNIT)


lidar = sens.ChLidarSensor(
    mesh_body,            
    20.0,                 
    lidar_pos,            
    1000,                 
    200,                  
    chrono.CH_C_PI,       
    chrono.CH_C_PI / 6.0, 
    100.0                 
)


lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)  
lidar.SetCollectionWindow(0.0)  


lidar.PushFilter(sens.ChFilterLidarReduce(1))  
lidar.PushFilter(sens.ChFilterLidarNoise(0.01))  
lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar Data"))  


if not os.path.exists("SENSOR_OUTPUT"):
    os.makedirs("SENSOR_OUTPUT")
lidar.PushFilter(sens.ChFilterSavePtCloud("SENSOR_OUTPUT/lidar_data"))


manager.AddSensor(lidar)




vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mesh with Lidar Sensor")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(8, 8, -6))
vis.AddTypicalLights()




step_size = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()
simulation_time = 0.0
orbit_radius = 10.0
orbit_height = 2.0
orbit_speed = 0.5  

while vis.Run():
    
    manager.Update()
    
    
    angle = simulation_time * orbit_speed
    new_x = orbit_radius * math.cos(angle)
    new_z = orbit_radius * math.sin(angle)
    
    
    new_pos = chrono.ChVectorD(new_x, orbit_height, new_z)
    
    
    direction = chrono.ChVectorD(0, 0, 0) - new_pos
    direction.Normalize()
    
    
    rotation = chrono.Q_from_AngZ(math.atan2(direction.z, direction.x))
    
    
    lidar_pos = chrono.ChFrameD(new_pos, rotation)
    lidar.SetOffsetPose(lidar_pos)
    
    
    if lidar.GetMostRecentBuffer():
        lidar_data = lidar.GetMostRecentBuffer().GetPointCloud()
        if lidar_data:
            print(f"Time: {simulation_time:.2f} - Lidar points: {len(lidar_data)}")
            if len(lidar_data) > 0:
                print(f"Sample point 0: ({lidar_data[0].x:.2f}, {lidar_data[0].y:.2f}, {lidar_data[0].z:.2f})")
    
    
    system.DoStepDynamics(step_size)
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    
    simulation_time += step_size
    realtime_timer.Spin(step_size)
    
    
    if simulation_time > 20:
        break


manager.ResetSensors()