import os
import math
import chrono
import chrono.irrlicht as chronoirr
import chrono.sensor as sens


chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))  

mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_body.AddAsset(mesh_shape)
system.Add(mesh_body)


lidar_body = chrono.ChBody()
lidar_body.SetBodyFixed(False)
lidar_body.SetPos(chrono.ChVectorD(5, 1, 0))
lidar_body.SetBodyType(chrono.ChBody.BODY_KINEMATIC)
system.Add(lidar_body)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 0), chrono.ChColor(0.8, 0.8, 0.8), 500)


lidar_offset = chrono.ChFrameD()
lidar = sens.ChLidarSensor(
    lidar_body,             
    30,                     
    lidar_offset,           
    1280,                   
    720,                    
    chrono.CH_C_PI / 1.5,  
    chrono.CH_C_PI / 4.5,  
    100.0                   
)


noise_model = sens.ChNoiseNormalDist(0.0, 0.02)
lidar.AddNoiseModel(noise_model)


lidar.PushFilter(sens.ChFilterVisualize(1280, 720, "Lidar Output"))


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))

manager.AddSensor(lidar)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.01
current_time = 0.0
radius = 5.0
angular_speed = 0.5  

def Q_from_angle_axis(angle, axis):
    quat = chrono.ChQuaternionD()
    quat.Q_from_AngAxis(angle, axis)
    return quat

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    angle = angular_speed * current_time
    x = radius * math.cos(angle)
    z = radius * math.sin(angle)
    lidar_body.SetPos(chrono.ChVectorD(x, 1.0, z))
    
    
    direction = (-lidar_body.GetPos()).GetNormalized()
    up = chrono.ChVectorD(0, 1, 0)
    rot = chrono.ChMatrix33D()
    rot.Set_A_axis(direction, up, direction.Cross(up))
    lidar_body.SetRot(rot.Get_A_quaternion())
    
    
    manager.Update()
    
    
    frame = lidar.GetMostRecentFrame()
    if frame.HasData():
        buffer = frame.GetDepthBuffer()
        print(f"Time: {current_time:.2f}s, Points: {buffer.GetWidth()}x{buffer.GetHeight()}")
        print("Sample distance:", buffer.GetPixel(640, 360))
    
    system.DoStepDynamics(time_step)
    current_time += time_step