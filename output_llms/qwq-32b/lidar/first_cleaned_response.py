import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Lidar Mesh Simulation')
vis.Initialize()
sys.SetVisualSystem(vis)


mesh_path = "path/to/mesh.obj"
mesh = chrono.ChTriangleMesh()
if not mesh.LoadWavefrontMesh(mesh_path):
    print("Error: Mesh file not found.")
    exit(1)


body = chrono.ChBody()
body.SetBodyFixed(True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
body.AddAsset(mesh_shape)
body.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))  
sys.Add(body)


sensor_mgr = sens.ChSensorManager(sys)
lidar = sens.ChLidar()
lidar.SetName("lidar_sensor")
lidar.SetPosition(chrono.ChVectorD(0, 0, 5))  
lidar.SetRotation(chrono.Q_from_Euler_123(0, 0, 0))  


lidar.SetHorizontalFOV(chrono.CH_C_PI / 2)  
lidar.SetVerticalFOV(chrono.CH_C_PI / 4)    
lidar.SetRange(50)  
lidar.SetNoiseModel(sens.ChLidarNoiseGaussian(0.05))  
lidar.SetNumberOfHorizontalBeams(360)
lidar.SetNumberOfVerticalBeams(1)
lidar.SetRenderPoints(True)  


sensor_mgr.AddSensor(lidar)
lidar.GetVisualizationAssets().Append(chrono.ChVisualizationLidar())


orbit_radius = 10.0  
angular_speed = 0.1  
current_angle = 0.0
step_size = 0.01  


while vis.Run():
    time = sys.GetChTime()
    
    
    current_angle += angular_speed * step_size
    x = orbit_radius * math.cos(current_angle)
    y = orbit_radius * math.sin(current_angle)
    z = 5.0  
    
    lidar.SetPosition(chrono.ChVectorD(x, y, z))
    
    
    lookat = chrono.ChVectorD(0, 0, 0)
    eye = chrono.ChVectorD(x, y, z)
    up = chrono.ChVectorD(0, 1, 0)  
    forward = lookat - eye
    forward.Normalize()
    
    right = forward.Cross(up)
    right.Normalize()
    up = right.Cross(forward)
    
    rot_matrix = chrono.ChMatrix33D()
    rot_matrix.Set_A_Xaxis(right)
    rot_matrix.Set_A_Yaxis(up)
    rot_matrix.Set_A_Zaxis(forward)
    rotation = chrono.Q_from_Matrix(rot_matrix)
    lidar.SetRotation(rotation)
    
    
    sensor_mgr.Update()
    buffer = lidar.GetBuffer()
    print(f"Time {time:.2f}s: Detected {len(buffer)} points")
    
    
    sys.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()