import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import pychrono.ros as ros
import numpy as np
import time


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(ground)


ground_vis = chrono.ChTriangleMeshShape()
ground_vis.SetMesh(chrono.GetChObjFileMesh("path/to/ground.obj"))  
ground.AddAsset(ground_vis)


platform = chrono.ChBody()
platform.SetPos(chrono.ChVectorD(0, 1, 0))
platform.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(platform)


link = chrono.ChLinkLockPrismatic()
link.Initialize(ground, platform, chrono.ChCoordinator(0, 1, 0))
my_system.Add(link)


def update_platform(time):
    platform_pos = chrono.ChVectorD(0, 1 + 0.5 * np.sin(time), 0)
    link.SetPrismaticDisplacement(platform_pos.y - 1)
    platform.SetPos(platform_pos)


sensor_manager = sensors.ChSensorManager(my_system)


ros_manager = ros.ChROSManager()
ros_manager.Initialize("chrono_simulation_node")



camera = sensors.ChCameraSensor(
    platform,                     
    10,                           
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),  
    640, 480,                     
    1.0472,                       
    0.1, 100                      
)
sensor_manager.AddSensor(camera)


lidar = sensors.ChLidarSensor(
    platform,
    10,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    360,                          
    180,                          
    0.1, 100,                     
    0.01                          
)
sensor_manager.AddSensor(lidar)


gps = sensors.ChGPSSensor(
    platform,
    10,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
sensor_manager.AddSensor(gps)


imu = sensors.ChIMUSensor(
    platform,
    100,                          
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
sensor_manager.AddSensor(imu)


ros_manager.RegisterSensor(camera, "/camera/image_raw", "sensor_msgs/Image")
ros_manager.RegisterSensor(lidar, "/lidar/points", "sensor_msgs/PointCloud2")
ros_manager.RegisterSensor(gps, "/gps/fix", "sensor_msgs/NavSatFix")
ros_manager.RegisterSensor(imu, "/imu/data", "sensor_msgs/Imu")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("PyChrono Sensor Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddTypicalLights()


time_step = 0.01
max_time = 10.0
current_time = 0


while vis.Run() and current_time < max_time:
    
    update_platform(current_time)

    
    sensor_manager.Update()

    
    ros_manager.Publish()

    
    my_system.DoStepDynamics(time_step)
    current_time += time_step

    
    time.sleep(time_step)

    
    print(f"Simulation time: {current_time:.2f} s")


ros_manager.Shutdown()