import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), '..', '..', 'data'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Lidar Simulation with Triangular Mesh")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 5))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChCameraVerticalDir::Z)


mesh_filename = "mesh.obj"  
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_filename, False, True)


body = chrono.ChBody()
body.SetBodyFixed(True)  
body.GetVisualModel().AddTriangleMesh(mesh)
body.GetCollisionModel().AddTriangleMesh(mesh)
system.Add(body)


sensor_manager = chrono.sensor.ChSensorManager(system)


lidar = chrono.sensor.ChLidarSensor()
lidar.SetName("LidarSensor")
lidar.SetUpdateRate(100)  
lidar.SetCollectInterval(0.01)  


lidar.SetRayCount(100, 100)  
lidar.SetRayLength(10.0)  
lidar.SetRaySpreadAngle(chrono.ChVector2d(60, 60))  


noise_model = chrono.sensor.ChGaussianNoiseModel()
noise_model.SetNoiseAmount(0.01)  
lidar.AddNoiseModel(noise_model)


lidar_visualization = chrono.sensor.ChLidarVisualization(system)
lidar_visualization.SetLidar(lidar)
lidar_visualization.SetPointSize(0.01)  
lidar_visualization.SetColor(chrono.ChColor(1, 0, 0))  


sensor_manager.AddSensor(lidar)


orbit_radius = 2.0
orbit_height = 1.0
angular_speed = 0.5  
current_angle = 0.0


time_step = 0.01
end_time = 10.0


while vis.Run():
    current_time = system.GetChTime()

    
    current_angle += angular_speed * time_step
    lidar_pos = chrono.ChVectorD(
        orbit_radius * np.cos(current_angle),
        orbit_height,
        orbit_radius * np.sin(current_angle)
    )

    
    lidar.SetOffsetPos(lidar_pos)
    lidar.SetOffsetRot(chrono.ChQuaternionD().Q_from_AngAxis(-current_angle, chrono.ChVectorD(0, 1, 0)))

    
    sensor_manager.Update()

    
    if lidar.GetBufferSize() > 0:
        buffer = lidar.GetMostRecentBuffer()
        print(f"Time: {current_time:.2f}s - Lidar points: {buffer.size()}")

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if current_time >= end_time:
        break


lidar.SaveBufferToFile("lidar_data.csv")