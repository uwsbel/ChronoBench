import math
import pychrono             as chrono
import pychrono.sensor       as sens
import pychrono.postprocess  as post           
import pychrono.ros          as chros          




chrono.SetChronoDataPath(chrono.GetChronoDataPath())

sys = chrono.ChSystemSMC()          
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

time_step       = 1.0 / 500.0       
end_time        = 10.0              




ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(False)          
ground.SetCollide(False)
sys.Add(ground)


ground.SetPos_dt(chrono.ChVectorD(1.0, 0, 0))   


vis_mesh = chrono.ChTriangleMeshConnected()
obj_path = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")
post.ChLoadOBJ(obj_path, vis_mesh)

mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(vis_mesh)
mesh_shape.SetName("chassis_mesh")
ground.AddAsset(mesh_shape)





manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVectorF(2, 2, 4), chrono.ChColor(1, 1, 1), 500)


ros_node_name = "chrono_sensor_node"
ros_bridge    = chros.ChROSBridge(manager, ros_node_name)





update_rate   = 30                  
lag           = 0                   
exposure_time = 1.0 / update_rate

sensor_offset = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5),   
                                chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,1,0)))


cam = sens.ChCameraSensor(ground, update_rate, sensor_offset,
                          1280, 720,      
                          math.radians(90))
cam.SetName("rgb_camera")
cam.PushFilter(sens.ChFilterRGBA8Access())                              
cam.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/camera/image"))    
manager.AddSensor(cam)


lidar = sens.ChLidarSensor(ground, 10, sensor_offset,                     
                           horiz_samples = 1024,
                           vert_samples  = 16,
                           horiz_fov     = math.radians(90),
                           max_vert_angle= math.radians(15))
lidar.SetName("lidar")
lidar.PushFilter(sens.ChFilterDIAccess())                                 
lidar.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/lidar/points"))
manager.AddSensor(lidar)


gps = sens.ChGPSSensor(ground, 1, sensor_offset)           
gps.SetName("gps")
gps.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/gps/fix"))
manager.AddSensor(gps)


accel = sens.ChAccelerometerSensor(ground, 200, sensor_offset)           
accel.SetName("accel")
accel.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/imu/accel"))
manager.AddSensor(accel)


gyro = sens.ChGyroscopeSensor(ground, 200, sensor_offset)
gyro.SetName("gyro")
gyro.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/imu/gyro"))
manager.AddSensor(gyro)


mag = sens.ChMagnetometerSensor(ground, 50, sensor_offset)
mag.SetName("mag")
mag.PushFilter(chros.ChFilterROSBridge(ros_bridge, "/imu/mag"))
manager.AddSensor(mag)




realtime_timer = chrono.ChRealtimeStepTimer()




print("Running simulation …  press Ctrl-C to quit")

sim_time = 0.0
try:
    while sim_time < end_time:
        
        manager.Update()
        ros_bridge.SpinOnce()          

        
        sys.DoStepDynamics(time_step)
        sim_time += time_step

        
        realtime_timer.Spin(time_step)

except KeyboardInterrupt:
    pass

print("Simulation finished.")
ros_bridge.Shutdown()