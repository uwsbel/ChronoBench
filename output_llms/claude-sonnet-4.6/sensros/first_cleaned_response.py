import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros
import math
import time




sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


sys.SetSolverMaxIterations(50)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)




ground_body = chrono.ChBody()
ground_body.SetName("ground")
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(False)  
ground_body.EnableCollision(False)


ground_visual = chrono.ChVisualShapeBox(2.0, 2.0, 0.1)
ground_visual.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground_body.AddVisualShape(ground_visual, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


try:
    mesh_shape = chrono.ChVisualShapeTriangleMesh()
    mesh = chrono.ChTriangleMeshConnected()
    
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj"),
                           False, True)
    mesh_shape.SetMesh(mesh)
    mesh_shape.SetName("terrain_mesh")
    mesh_shape.SetMutable(False)
    ground_body.AddVisualShape(mesh_shape)
except Exception as e:
    print(f"Mesh loading skipped (file not found): {e}")
    
    fallback_visual = chrono.ChVisualShapeBox(10.0, 10.0, 0.2)
    fallback_visual.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    ground_body.AddVisualShape(fallback_visual, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1)))

sys.AddBody(ground_body)





class MotionFunction(chrono.ChFunctionSetpoint):
    def __init__(self):
        super().__init__()
        self.amplitude = 2.0   
        self.frequency = 0.5   
        
    def GetVal(self, t):
        return self.amplitude * math.sin(2 * math.pi * self.frequency * t)
    
    def GetDer(self, t):
        return self.amplitude * 2 * math.pi * self.frequency * math.cos(
            2 * math.pi * self.frequency * t)


ground_body.SetPos(chrono.ChVector3d(0, 0, 1.0))  



ground_body.SetMass(100.0)
inertia = chrono.ChVector3d(10, 10, 10)
ground_body.SetInertiaXX(inertia)




manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    5000
)
manager.SetKeyframeSizeFromTimeStep(0.01, 1.0 / 10)


b = sens.Background()
b.color_horizon = chrono.ChVector3f(0.6, 0.7, 0.8)
b.color_zenith = chrono.ChVector3f(0.4, 0.5, 0.6)
b.mode = sens.BackgroundMode_GRADIENT
manager.scene.SetBackground(b)




sensor_offset_pos = chrono.ChVector3d(0, 0, 0.5)
sensor_offset_rot = chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
sensor_frame = chrono.ChFramed(sensor_offset_pos, sensor_offset_rot)




cam_update_rate = 30       
cam_width = 1280
cam_height = 720
cam_fov = math.pi / 3.0   
cam_lag = 0.0
cam_collection_time = 1.0 / cam_update_rate


cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-1.0, 0, 1.5),
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0))
)

camera = sens.ChCameraSensor(
    ground_body,           
    cam_update_rate,       
    cam_offset,            
    cam_width,             
    cam_height,            
    cam_fov,               
)
camera.SetName("CameraSensor")
camera.SetLag(cam_lag)
camera.SetCollectionWindow(cam_collection_time)


camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Camera View"))

camera.PushFilter(sens.ChFilterRGBA8Access())

manager.AddSensor(camera)




lidar_update_rate = 10     
lidar_lag = 0.0
lidar_collection_time = 1.0 / lidar_update_rate
lidar_samples_per_beam = 1
horizontal_samples = 900
vertical_samples = 16
horizontal_fov = 2 * math.pi         
min_vert_angle = -math.pi / 12       
max_vert_angle = math.pi / 12        
lidar_max_distance = 100.0
lidar_min_distance = 0.1

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
)

lidar = sens.ChLidarSensor(
    ground_body,
    lidar_update_rate,
    lidar_offset,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    lidar_max_distance,
)
lidar.SetName("LidarSensor")
lidar.SetLag(lidar_lag)
lidar.SetCollectionWindow(lidar_collection_time)


lidar.PushFilter(sens.ChFilterDIAccess())           
lidar.PushFilter(sens.ChFilterPCfromDepth())        
lidar.PushFilter(sens.ChFilterXYZIAccess())         
lidar.PushFilter(sens.ChFilterVisualize(
    horizontal_samples, vertical_samples, "LiDAR Depth View"
))

manager.AddSensor(lidar)




gps_update_rate = 10    
gps_lag = 0.0
gps_collection_time = 1.0 / gps_update_rate


gps_reference = chrono.ChVector3d(43.073268, -89.400636, 260.0)

gps_noise_model = sens.ChNoiseNormal(
    chrono.ChVector3d(0, 0, 0),    
    chrono.ChVector3d(0.01, 0.01, 0.01)  
)

gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.3),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
)

gps = sens.ChGPSSensor(
    ground_body,
    gps_update_rate,
    gps_offset,
    gps_reference,
    gps_noise_model
)
gps.SetName("GPSSensor")
gps.SetLag(gps_lag)
gps.SetCollectionWindow(gps_collection_time)
gps.PushFilter(sens.ChFilterGPSAccess())

manager.AddSensor(gps)




imu_update_rate = 100   
imu_lag = 0.0
imu_collection_time = 1.0 / imu_update_rate

imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
)


acc_noise_model = sens.ChNoiseNormalDrift(
    imu_update_rate,             
    chrono.ChVector3d(0, 0, 0),  
    chrono.ChVector3d(0.001, 0.001, 0.001),  
    0.0,                         
    0.1                          
)

accelerometer = sens.ChAccelerometerSensor(
    ground_body,
    imu_update_rate,
    imu_offset,
    acc_noise_model
)
accelerometer.SetName("AccelerometerSensor")
accelerometer.SetLag(imu_lag)
accelerometer.SetCollectionWindow(imu_collection_time)
accelerometer.PushFilter(sens.ChFilterAccelAccess())

manager.AddSensor(accelerometer)


gyro_noise_model = sens.ChNoiseNormalDrift(
    imu_update_rate,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0.001, 0.001, 0.001),
    0.0,
    0.1
)

gyroscope = sens.ChGyroscopeSensor(
    ground_body,
    imu_update_rate,
    imu_offset,
    gyro_noise_model
)
gyroscope.SetName("GyroscopeSensor")
gyroscope.SetLag(imu_lag)
gyroscope.SetCollectionWindow(imu_collection_time)
gyroscope.PushFilter(sens.ChFilterGyroAccess())

manager.AddSensor(gyroscope)


mag_noise_model = sens.ChNoiseNormal(
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(0.001, 0.001, 0.001)
)

mag_reference = chrono.ChVector3d(43.073268, -89.400636, 260.0)

magnetometer = sens.ChMagnetometerSensor(
    ground_body,
    imu_update_rate,
    imu_offset,
    mag_noise_model,
    mag_reference
)
magnetometer.SetName("MagnetometerSensor")
magnetometer.SetLag(imu_lag)
magnetometer.SetCollectionWindow(imu_collection_time)
magnetometer.PushFilter(sens.ChFilterMagnetAccess())

manager.AddSensor(magnetometer)




ros_manager = chros.ChROSManager()


ros_manager.RegisterHandler(chros.ChROSClockHandler())


ros_manager.RegisterHandler(
    chros.ChROSCameraHandler(
        cam_update_rate,
        camera,
        "~/output/camera/image_raw"
    )
)


ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        lidar,
        "~/output/lidar/point_cloud"
    )
)


ros_manager.RegisterHandler(
    chros.ChROSGPSHandler(
        gps_update_rate,
        gps,
        "~/output/gps/data"
    )
)


ros_manager.RegisterHandler(
    chros.ChROSAccelerometerHandler(
        imu_update_rate,
        accelerometer,
        "~/output/imu/accelerometer"
    )
)


ros_manager.RegisterHandler(
    chros.ChROSGyroscopeHandler(
        imu_update_rate,
        gyroscope,
        "~/output/imu/gyroscope"
    )
)


ros_manager.RegisterHandler(
    chros.ChROSMagnetometerHandler(
        imu_update_rate,
        magnetometer,
        "~/output/imu/magnetometer"
    )
)


ros_manager.Initialize()




step_size = 1e-3        
sim_time = 0.0
end_time = 30.0         
render_step_size = 1.0 / 25  


realtime_timer = chrono.ChRealtimeStepTimer()

print("=" * 60)
print(" PyChrono Simulation with Sensors and ROS Integration")
print("=" * 60)
print(f"  Step size:      {step_size * 1000:.1f} ms")
print(f"  Simulation end: {end_time:.1f} s")
print(f"  Camera:         {cam_width}x{cam_height} @ {cam_update_rate} Hz")
print(f"  LiDAR:          {horizontal_samples}x{vertical_samples} @ {lidar_update_rate} Hz")
print(f"  GPS:            @ {gps_update_rate} Hz")
print(f"  IMU:            @ {imu_update_rate} Hz")
print("=" * 60)

step_count = 0
motion_amplitude = 2.0
motion_frequency = 0.3

while sim_time < end_time:
    
    
    
    x_pos = motion_amplitude * math.sin(2 * math.pi * motion_frequency * sim_time)
    y_pos = motion_amplitude * 0.5 * math.cos(2 * math.pi * motion_frequency * sim_time)
    z_pos = 1.0 + 0.3 * math.sin(2 * math.pi * 0.8 * sim_time)

    x_vel = (motion_amplitude * 2 * math.pi * motion_frequency *
             math.cos(2 * math.pi * motion_frequency * sim_time))
    y_vel = (-motion_amplitude * 0.5 * 2 * math.pi * motion_frequency *
              math.sin(2 * math.pi * motion_frequency * sim_time))
    z_vel = (0.3 * 2 * math.pi * 0.8 *
             math.cos(2 * math.pi * 0.8 * sim_time))

    ground_body.SetPos(chrono.ChVector3d(x_pos, y_pos, z_pos))
    ground_body.SetPosDt(chrono.ChVector3d(x_vel, y_vel, z_vel))

    
    roll = 0.1 * math.sin(2 * math.pi * 0.4 * sim_time)
    pitch = 0.05 * math.cos(2 * math.pi * 0.3 * sim_time)
    yaw = 2 * math.pi * 0.1 * sim_time  

    rot = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
    rot_roll = chrono.QuatFromAngleAxis(roll, chrono.ChVector3d(1, 0, 0))
    rot_pitch = chrono.QuatFromAngleAxis(pitch, chrono.ChVector3d(0, 1, 0))
    ground_body.SetRot(rot * rot_pitch * rot_roll)

    
    
    
    manager.Update()

    
    
    
    if not ros_manager.Update(sim_time, step_size):
        print("ROS manager update failed or ROS shutdown requested. Exiting...")
        break

    
    
    
    sys.DoStepDynamics(step_size)
    sim_time += step_size
    step_count += 1

    
    
    
    realtime_timer.Spin(step_size)

    
    
    
    if step_count % 1000 == 0:
        body_pos = ground_body.GetPos()
        body_vel = ground_body.GetPosDt()
        
        print(f"  t={sim_time:6.2f}s | "
              f"Pos=({body_pos.x:6.2f}, {body_pos.y:6.2f}, {body_pos.z:6.2f}) | "
              f"Vel=({body_vel.x:5.2f}, {body_vel.y:5.2f}, {body_vel.z:5.2f})")

        
        try:
            gps_data = gps.GetMostRecentGPSBuffer()
            if gps_data.HasData():
                gps_vals = gps_data.GetGPSData()
                print(f"    GPS: lat={gps_vals[0]:.6f}, "
                      f"lon={gps_vals[1]:.6f}, "
                      f"alt={gps_vals[2]:.2f}")
        except Exception:
            pass

        try:
            acc_data = accelerometer.GetMostRecentAccelBuffer()
            if acc_data.HasData():
                acc_vals = acc_data.GetAccelData()
                print(f"    Accel: ({acc_vals[0]:.3f}, "
                      f"{acc_vals[1]:.3f}, "
                      f"{acc_vals[2]:.3f}) m/s²")
        except Exception:
            pass

        try:
            gyro_data = gyroscope.GetMostRecentGyroBuffer()
            if gyro_data.HasData():
                gyro_vals = gyro_data.GetGyroData()
                print(f"    Gyro:  ({gyro_vals[0]:.4f}, "
                      f"{gyro_vals[1]:.4f}, "
                      f"{gyro_vals[2]:.4f}) rad/s")
        except Exception:
            pass

print("=" * 60)
print(f" Simulation complete. Total steps: {step_count}")
print(f" Final simulation time: {sim_time:.3f}s")
print("=" * 60)