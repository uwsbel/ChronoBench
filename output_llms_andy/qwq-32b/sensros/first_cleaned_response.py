import pychrono as chrono
import pychrono.sensor as sensors
import pychrono.irrlicht as chronoirr
import pyros as ros  


my_system = chrono.ChSystemNSC()
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetMaxItersSolverSpeed(100)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(False)  
ground.SetMass(1000)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetPos_dt(chrono.ChVectorD(1, 0, 0))  
my_system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 10, 0.1))
ground.AddAsset(ground_shape)


sensor_mgr = sensors.ChSensorManager()
sensor_mgr.SetCoordinateSystem(chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))


ros_node = ros.Ros()
ros_node.init('chronosim')
ros_mgr = sensors.ChRosSensorManager()
ros_mgr.SetRosInterface(ros_node)



cam = sensors.ChCameraSensor()
cam.SetName("camera")
cam.SetPos(chrono.ChVectorD(0, 0, 1))  
cam.SetDir(chrono.ChVectorD(0, -1, 0))  
cam.SetFov(60)
cam.SetImageWidth(640)
cam.SetImageHeight(480)
cam.SetPositionRelative(ground)
sensor_mgr.AddSensor(cam)
ros_mgr.RegisterSensor(cam, '/camera/image', 'sensor_msgs/Image')


lidar = sensors.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPos(chrono.ChVectorD(0, 0, 1))
lidar.SetDir(chrono.ChVectorD(0, -1, 0))
lidar.SetHorizontalResolution(0.1)
lidar.SetVerticalResolution(0.1)
lidar.SetRange(50)
lidar.SetPositionRelative(ground)
sensor_mgr.AddSensor(lidar)
ros_mgr.RegisterSensor(lidar, '/lidar/points', 'sensor_msgs/PointCloud2')


gps = sensors.ChGpsSensor()
gps.SetName("gps")
gps.SetUpdateRate(1.0/10)  
gps.SetPositionRelative(ground)
sensor_mgr.AddSensor(gps)
ros_mgr.RegisterSensor(gps, '/gps/fix', 'sensor_msgs/NavSatFix')


imu = sensors.ChImuSensor()
imu.SetName("imu")
imu.SetPositionRelative(ground)
sensor_mgr.AddSensor(imu)
ros_mgr.RegisterSensor(imu, '/imu/data', 'sensor_msgs/Imu')


class MagnetometerSensor(sensors.ChSensor):
    def __init__(self, body):
        super().__init__()
        self.body = body
        self.magnetic_field = chrono.ChVectorD(0, 0, 100000)  

    def Update(self):
        rotation = self.body.GetFrame_REF_to_abs().Get_A_Rotation()
        magnetic_body = rotation.Transpose() * self.magnetic_field
        self.SetData(magnetic_body)

mag = MagnetometerSensor(ground)
mag.SetName("magnetometer")
sensor_mgr.AddSensor(mag)
ros_mgr.RegisterSensor(mag, '/magnetometer/data', 'sensor_msgs/MagneticField')


step_size = 0.01
sim_time = 10.0


start_time = time.time()
while my_system.GetChTime() < sim_time:
    
    sensor_mgr.Update()
    
    
    my_system.DoStepDynamics(step_size)
    
    
    ground.SetPos_dt(chrono.ChVectorD(1, 0, 0))  
    
    
    ros_node.spin_once()
    
    
    elapsed = time.time() - start_time
    sleep_time = step_size - (time.time() - start_time)
    if sleep_time > 0:
        time.sleep(sleep_time)
    else:
        print("Simulation is falling behind real-time")

print("Simulation completed.")