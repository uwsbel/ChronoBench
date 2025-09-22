import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
application = chronoirr.ChIrrApp(system, 'PyChrono Lidar Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0, 3, 6), chronoirr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh('path_to_mesh.obj', True, True)  


body = chrono.ChBody()
body.SetBodyFixed(True)
body.GetCollisionModel().ClearModel()
body.GetCollisionModel().AddTriangleMesh(mesh, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1), 0.001)
body.SetCollide(False)


vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh)
vis_shape.SetOpacity(1.0)
body.AddVisualShape(vis_shape)

system.Add(body)


manager = sens.ChSensorManager(system)

lidar = sens.ChLidarSensor(
    body,                    
    100,                   
    chrono.ChFrameD(),     
    360,                   
    30,                    
    chrono.CH_C_PI,        
    chrono.CH_C_PI / 4,    
    100,                   
    sens.LidarBeamShape_RECTANGULAR,
    2,                     
    0.003,                 
    0.003                  
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1 / 100)


lidar.AddFilter(sens.ChFilterNoiseLidarNormal(0.01))  
lidar.AddFilter(sens.ChFilterVisual())               
lidar.AddFilter(sens.ChFilterPCwrite())              

manager.AddSensor(lidar)


time_step = 0.01
time = 0
orbit_radius = 5
orbit_height = 2
omega = 0.5  


while application.GetDevice().run():
    
    theta = omega * time
    x = orbit_radius * math.cos(theta)
    z = orbit_radius * math.sin(theta)
    y = orbit_height
    lidar_pos = chrono.ChVectorD(x, y, z)
    
    
    dir_to_origin = -lidar_pos.GetNormalized()
    rot = chrono.ChQuaternionFromTwoVectors(chrono.ChVectorD(1, 0, 0), dir_to_origin)
    
    
    lidar.SetOffsetPose(chrono.ChFrameD(lidar_pos, rot))
    
    
    system.DoStepDynamics(time_step)
    manager.Update()
    
    
    if lidar.HasData():
        data = lidar.GetMostRecentData()
        point_cloud = data.GetPointcloud()
        print(f"Time: {time:.2f}, Point cloud size: {point_cloud.size()}")
        if point_cloud.size() > 0:
            first_point = point_cloud[0]
            print(f"  First point: ({first_point.x:.2f}, {first_point.y:.2f}, {first_point.z:.2f})")
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    time += time_step