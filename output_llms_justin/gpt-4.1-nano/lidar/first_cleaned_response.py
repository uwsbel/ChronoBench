import pychrono as chrono
from pychrono.sensor import ChSensorManager, ChLidarSensor, ChSensorFilterNoiseModel, ChSensorFilterVisualizer, ChSensorSave
import math


chrono.SetChronoDataPath('your/data/path')  
system = chrono.ChSystemNSC()



mesh_filename = 'path_to_mesh.obj'
mesh_body = chrono.ChBodySimple()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)


mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(chrono.ChTriangleMesh().LoadWavefront(mesh_filename))
mesh_body.AddVisualShape(mesh_shape)


collision_shape = chrono.ChTriangleMeshShape()
collision_shape.SetMesh(chrono.ChTriangleMesh().LoadWavefront(mesh_filename))
mesh_body.GetCollisionModel().ClearModel()
collision_shape.AddTriangleMeshGeometry(chrono.VNULL,  
                                       chrono.QUNIT,  
                                       True)        
mesh_body.GetCollisionModel().AddTriangleMesh(collision_shape, False, False)
mesh_body.GetCollisionModel().BuildModel()
mesh_body.SetCollide(True)

system.Add(mesh_body)


sensor_manager = ChSensorManager()
sensor_manager.scene = system


lidar_name = 'my_lidar'
lidar_range = 50.0  
lidar_h_fov = math.radians(90)  
lidar_v_fov = math.radians(30)  
lidar_h_res = 800  
lidar_v_res = 200  

lidar_pose = chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT)  
lidar_sensor = ChLidarSensor(
    mesh_body,  
    lidar_pose,
    lidar_range,
    lidar_h_fov,
    lidar_v_fov,
    lidar_h_res,
    lidar_v_res
)


lidar_sensor.GetScanIO().EnableFilter(ChSensorFilterNoiseModel())
lidar_sensor.GetScanIO().EnableFilter(ChSensorFilterVisualizer())


lidar_sensor.GetScanIO().EnableSave(
    ChSensorSave.Format.JSON,         
    "lidar_output.json"              
)

sensor_manager.AddSensor(lidar_sensor)


time_end = 10  
dt = 0.01     
t = 0

import matplotlib.pyplot as plt

while t < time_end:
    
    radius = 2.0  
    angular_speed = math.pi / 5  
    angle = angular_speed * t

    
    x_pos = radius * math.cos(angle)
    y_pos = 1.5  
    z_pos = radius * math.sin(angle)
    new_pos = chrono.ChVectorD(x_pos, y_pos, z_pos)
    new_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))  

    
    lidar_sensor.Pose = chrono.ChFrameD(new_pos, new_rot)

    
    sensor_manager.Synchronize(t)
    sensor_manager.Advance(dt)

    
    if lidar_sensor.GetScanIO().HasValidFrame():
        
        points = lidar_sensor.GetScanIO().GetPointCloud()
        print(f"Time {t:.2f}s: Lidar point cloud with {len(points)} points.")

    system.DoStepDynamics(dt)
    t += dt