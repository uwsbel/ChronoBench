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


mesh_path = "path/to/your/mesh.obj"  
mesh = chrono.ChTriangleMesh()
if not mesh.LoadWavefrontMesh(mesh_path):
    print("Error: Could not load mesh file.")
    exit(1)

mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_body.AddAsset(mesh_shape)
mesh_body.AddAsset(chrono.ChColorAsset(0.5, 0.5, 0.5))  
sys.Add(mesh_body)


sensor_mgr = sens.ChSensorManager(sys)

lidar = sens.ChLidar()
lidar.SetName("lidar_sensor")
lidar.SetRange(50)  
lidar.SetHorizontalFOV(chrono.CH_C_PI / 2)  
lidar.SetVerticalFOV(chrono.CH_C_PI / 4)    
lidar.SetNumberOfHorizontalBeams(360)
lidar.SetNumberOfVerticalBeams(1)
lidar.SetNoiseModel(sens.ChLidarNoiseGaussian(0.05))  
lidar.SetRenderPoints(True)  

sensor_mgr.AddSensor(lidar)


orbit_radius = 10.0  
angular_speed = 0.1  
current_angle = 0.0
z_height = 5.0  


while vis.Run():
    time = sys.GetChTime()
    
    
    current_angle += angular_speed * sys.GetTimer().GetDeltaT()
    x = orbit_radius * math.cos(current_angle)
    y = orbit_radius * math.sin(current_angle)
    lidar.SetPosition(chrono.ChVectorD(x, y, z_height))
    
    
    eye = chrono.ChVectorD(x, y, z_height)
    lookat = chrono.ChVectorD(0, 0, 0)
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
    lidar.SetRotation(chrono.Q_from_Matrix(rot_matrix))
    
    
    sensor_mgr.Update()
    buffer = lidar.GetBuffer()
    print(f"Time {time:.2f}s: {len(buffer)} points detected")
    
    
    sys.DoStepDynamics(0.01)  
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()